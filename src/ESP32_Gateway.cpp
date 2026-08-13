#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// =====================================================
// WIFI CONFIG
// =====================================================
const char* ssid     = "AL";
const char* password = "AL280908";
#define ESPNOW_CHANNEL 6

// UART serial from ESP8266 receiver.
#define UART_RX_PIN 16
#define UART_TX_PIN 17
#define UART_BAUD 115200

// =====================================================
// MQTT CONFIG (HiveMQ - Sangat Cepat & Ringan)
// =====================================================
const char* mqtt_server = "a4e9379a555f47669c90f4c69b75eeda.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "NexaGrowv2";
const char* mqtt_password = "NexaGrow12345";
const char* mqtt_client_id = "ESP32_Gateway_Nexa";

WiFiClientSecure secureClient; 
PubSubClient mqttClient(secureClient);

// =====================================================
// SENSOR DATA PACKET
// =====================================================
typedef struct __attribute__((packed)) {
  uint8_t  nodeID;
  float    soil;
  float    temperature;
  float    humidity;
  float    battery;
  uint32_t counter;
} SensorPacket;

// =====================================================
// DATA BUFFER — Antrean saat internet/MQTT putus
// =====================================================
#define BUFFER_SIZE 30
String dataBuffer[BUFFER_SIZE];
String topicBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferRead = 0;
int bufferCount = 0;

void bufferAdd(String topic, String json) {
  if (bufferCount >= BUFFER_SIZE) {
    dataBuffer[bufferRead] = "";
    topicBuffer[bufferRead] = "";
    bufferRead = (bufferRead + 1) % BUFFER_SIZE;
    bufferCount--;
  }
  dataBuffer[bufferHead] = json;
  topicBuffer[bufferHead] = topic;
  bufferHead = (bufferHead + 1) % BUFFER_SIZE;
  bufferCount++;
}

// =====================================================
// HELPER
// =====================================================
void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}

// =====================================================
// PUBLISH DATA (MURNI MQTT - TIDAK ADA LAG HTTP)
// =====================================================
void processData(int nodeId, String jsonPayload) {
  String topic = "sproutai/sensor/node/" + String(nodeId);
  String topicData = "sproutai/sensor/data";

  if (mqttClient.connected()) {
    mqttClient.publish(topic.c_str(), jsonPayload.c_str(), false);
    mqttClient.publish(topicData.c_str(), jsonPayload.c_str(), false);
    Serial.print("[MQTT] Sent Node "); Serial.println(nodeId);
  } else {
    Serial.print("[MQTT] Offline - Buffering Node "); Serial.println(nodeId);
    bufferAdd(topic, jsonPayload);
  }
}

void flushBuffer() {
  if (WiFi.status() != WL_CONNECTED || !mqttClient.connected() || bufferCount == 0) return;
  int sent = 0;
  int initialCount = bufferCount;
  
  for (int i = 0; i < initialCount; ++i) {
    if (!mqttClient.connected()) break; 

    String p = dataBuffer[bufferRead];
    String t = topicBuffer[bufferRead];
    
    if (p.length() > 0 && t.length() > 0) {
      if (mqttClient.publish(t.c_str(), p.c_str(), false)) {
        mqttClient.publish("sproutai/sensor/data", p.c_str(), false);
        dataBuffer[bufferRead] = "";
        topicBuffer[bufferRead] = "";
        bufferRead = (bufferRead + 1) % BUFFER_SIZE;
        bufferCount--;
        sent++;
      } else {
        break; 
      }
    } else {
      bufferRead = (bufferRead + 1) % BUFFER_SIZE;
      bufferCount--;
    }
    delay(20); // Jeda sangat kecil
  }
}

// =====================================================
// SERIAL2 (ESP8266 Receiver) HANDLING
// =====================================================
static String serial2Line = "";

void readSerial2() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') continue;
    
    if (c == '\n') {
      serial2Line.trim(); 
      int startIdx = serial2Line.indexOf('{');
      int endIdx = serial2Line.lastIndexOf('}');
      
      if (startIdx >= 0 && endIdx > startIdx) {
        String cleanJson = serial2Line.substring(startIdx, endIdx + 1);
        
        int nodeId = 1; // Default
        int idIdx = cleanJson.indexOf("\"node_id\":");
        if(idIdx > 0) nodeId = cleanJson.substring(idIdx + 10).toInt();
        
        processData(nodeId, cleanJson);
        
      }
      serial2Line = "";
    } else {
      serial2Line += c;
    }
    if (serial2Line.length() > 512) serial2Line = "";
  }
}

// =====================================================
// WIFI & MQTT MANAGEMENT
// =====================================================
bool wifiWasConnected = false;
unsigned long lastReconnectAttempt = 0;

void ensureWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("[WiFi] Connected! IP: ");
      Serial.println(WiFi.localIP());
      secureClient.setInsecure(); // SSL MQTT
    }
    return;
  }
  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("[WiFi] Lost. Auto-reconnecting...");
  }
  unsigned long now = millis();
  if (now - lastReconnectAttempt >= 10000) {
    WiFi.reconnect();
    lastReconnectAttempt = now;
  }
}

void ensureMqttConnection() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqttClient.connected()) {
    Serial.print("[MQTT] Connecting to HiveMQ... ");
    if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("connected!");
    } else {
      Serial.println("failed!");
    }
  }
}

// =====================================================
// ESP-NOW DATA HANDLING
// =====================================================
typedef struct {
  SensorPacket packet;
  uint8_t mac[6];
} QueueItem;

static QueueHandle_t sensorQueue = NULL;
#define SENSOR_QUEUE_LEN 10

#if defined(ESP32) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac_addr = info->src_addr;
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
#endif
  if (len != sizeof(SensorPacket)) return;
  QueueItem item;
  memcpy(&item.packet, data, sizeof(SensorPacket));
  memcpy(item.mac, mac_addr, 6);
  if (xQueueSend(sensorQueue, &item, 0) != pdTRUE) {
    Serial.println("[ESP-NOW] queue full");
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("================================");
  Serial.println("   NexaGrow ESP32 Gateway");
  Serial.println("   PURE MQTT MODE");
  Serial.println("================================");

  sensorQueue = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(QueueItem));

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && millis() < 15000) {
    delay(500); Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiWasConnected = true;
    secureClient.setInsecure(); 
  }

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setKeepAlive(15); 
  // Set buffer size MQTT lebih besar agar tidak mudah gagal
  mqttClient.setBufferSize(512);

  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  int espNowChannel = WiFi.status() == WL_CONNECTED ? WiFi.channel() : ESPNOW_CHANNEL;
  if (WiFi.status() != WL_CONNECTED) {
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  }
  
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("[ESP-NOW] Receiver READY");
  }
}

// =====================================================
// LOOP
// =====================================================
unsigned long lastMqttCheck = 0;
unsigned long lastBufferFlush = 0;

void loop() {
  ensureWiFiConnection();

  if (millis() - lastMqttCheck >= 5000) {
    ensureMqttConnection();
    lastMqttCheck = millis();
  }
  
  if (mqttClient.connected()) {
    mqttClient.loop(); 
  }

  readSerial2();

  if (millis() - lastBufferFlush >= 30000) {
    flushBuffer();
    lastBufferFlush = millis();
  }

  QueueItem item;
  while (xQueueReceive(sensorQueue, &item, 0) == pdTRUE) {
    SensorPacket data = item.packet;
    
    String json = "{";
    json += "\"node_id\":" + String(data.nodeID) + ",";
    json += "\"temperature\":" + (isnan(data.temperature) ? "null" : String(data.temperature, 2)) + ",";
    json += "\"humidity\":" + (isnan(data.humidity) ? "null" : String(data.humidity, 2)) + ",";
    json += "\"soil_moisture\":" + (isnan(data.soil) ? "null" : String(data.soil, 2));
    json += "}";

    processData(data.nodeID, json);
  }
}