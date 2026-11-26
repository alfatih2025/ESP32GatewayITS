#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h> 
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
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
// CACHED BMKG WEATHER SNAPSHOT
// =====================================================
static const char* BMKG_TOPIC = "sproutai/weather/bmkg";
static const unsigned long BMKG_STALE_AFTER_SECONDS = 6UL * 60UL * 60UL;
static const size_t BMKG_MAX_PAYLOAD_SIZE = 512;

struct BmkgSnapshot {
  bool valid;
  char locationCode[33];
  char fetchedAt[32];
  int64_t fetchedEpoch;
  float temperatureC;
  float humidityPct;
  float windSpeedKmh;
  float rainAmountMm;
  float rainChancePct;
};

static BmkgSnapshot bmkgSnapshot = {};
static Preferences weatherPreferences;
static bool weatherStorageReady = false;

bool validLocationCode(const char* value) {
  if (value == NULL) return false;
  size_t length = strlen(value);
  if (length == 0 || length > 32) return false;
  for (size_t i = 0; i < length; ++i) {
    char c = value[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || c == '.')) return false;
  }
  return true;
}

bool isLeapYear(int year) {
  return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
  const unsigned dayOfYear = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
  return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(dayOfEra) - 719468;
}

bool parseIsoTimestamp(const char* value, int64_t& epoch) {
  if (value == NULL) return false;
  size_t length = strlen(value);
  if (length < 20 || length >= 40) return false;
  if (value[4] != '-' || value[7] != '-' || value[10] != 'T' ||
      value[13] != ':' || value[16] != ':') return false;

  for (int i = 0; i < 19; ++i) {
    if (i == 4 || i == 7 || i == 10 || i == 13 || i == 16) continue;
    if (value[i] < '0' || value[i] > '9') return false;
  }

  int year = (value[0] - '0') * 1000 + (value[1] - '0') * 100 +
             (value[2] - '0') * 10 + value[3] - '0';
  int month = (value[5] - '0') * 10 + value[6] - '0';
  int day = (value[8] - '0') * 10 + value[9] - '0';
  int hour = (value[11] - '0') * 10 + value[12] - '0';
  int minute = (value[14] - '0') * 10 + value[15] - '0';
  int second = (value[17] - '0') * 10 + value[18] - '0';

  if (year < 2020 || month < 1 || month > 12 || hour > 23 ||
      minute > 59 || second > 59) return false;
  const int daysInMonth[] = {31, isLeapYear(year) ? 29 : 28, 31, 30, 31, 30,
                             31, 31, 30, 31, 30, 31};
  if (day < 1 || day > daysInMonth[month - 1]) return false;

  size_t zoneIndex = 19;
  if (value[zoneIndex] == '.') {
    size_t fractionStart = ++zoneIndex;
    while (zoneIndex < length && value[zoneIndex] >= '0' && value[zoneIndex] <= '9')
      ++zoneIndex;
    if (zoneIndex == fractionStart || zoneIndex - fractionStart > 9) return false;
  }

  int offsetSeconds = 0;
  if (value[zoneIndex] == 'Z' && zoneIndex + 1 == length) {
    // UTC timestamp.
  } else {
    if ((value[zoneIndex] != '+' && value[zoneIndex] != '-') ||
        zoneIndex + 6 != length || value[zoneIndex + 3] != ':') return false;
    if (value[zoneIndex + 1] < '0' || value[zoneIndex + 1] > '9' ||
        value[zoneIndex + 2] < '0' || value[zoneIndex + 2] > '9' ||
        value[zoneIndex + 4] < '0' || value[zoneIndex + 4] > '9' ||
        value[zoneIndex + 5] < '0' || value[zoneIndex + 5] > '9') return false;
    int offsetHour = (value[zoneIndex + 1] - '0') * 10 + value[zoneIndex + 2] - '0';
    int offsetMinute = (value[zoneIndex + 4] - '0') * 10 + value[zoneIndex + 5] - '0';
    if (offsetHour > 14 || offsetMinute > 59 ||
        (offsetHour == 14 && offsetMinute != 0)) return false;
    offsetSeconds = (offsetHour * 60 + offsetMinute) * 60;
    if (value[zoneIndex] == '-') offsetSeconds = -offsetSeconds;
  }

  epoch = daysFromCivil(year, static_cast<unsigned>(month), static_cast<unsigned>(day)) * 86400 +
          hour * 3600 + minute * 60 + second - offsetSeconds;
  return true;
}

bool parseBmkgPayload(const uint8_t* payload, size_t length, BmkgSnapshot& result) {
  if (payload == NULL || length == 0 || length > BMKG_MAX_PAYLOAD_SIZE) return false;
  JsonDocument document;
  if (deserializeJson(document, payload, length) ||
      !document.is<JsonObject>() ||
      !document["schema_version"].is<int>() ||
      document["schema_version"].as<int>() != 1) return false;

  const char* locationCode = document["location_code"].as<const char*>();
  const char* fetchedAt = document["fetched_at"].as<const char*>();
  if (!validLocationCode(locationCode) || fetchedAt == NULL ||
      strlen(fetchedAt) >= sizeof(result.fetchedAt)) return false;

  JsonVariantConst temperature = document["temperature_c"];
  JsonVariantConst humidity = document["humidity_pct"];
  JsonVariantConst windSpeed = document["wind_speed_kmh"];
  JsonVariantConst rainAmount = document["rain_amount_mm"];
  JsonVariantConst rainChance = document["rain_chance_pct"];
  if ((!temperature.is<int>() && !temperature.is<double>()) ||
      (!humidity.is<int>() && !humidity.is<double>()) ||
      (!windSpeed.is<int>() && !windSpeed.is<double>()) ||
      (!rainAmount.is<int>() && !rainAmount.is<double>()) ||
      (!rainChance.is<int>() && !rainChance.is<double>())) return false;

  float temperatureC = temperature.as<float>();
  float humidityPct = humidity.as<float>();
  float windSpeedKmh = windSpeed.as<float>();
  float rainAmountMm = rainAmount.as<float>();
  float rainChancePct = rainChance.as<float>();
  if (!isfinite(temperatureC) || temperatureC < -50.0f || temperatureC > 60.0f ||
      !isfinite(humidityPct) || humidityPct < 0.0f || humidityPct > 100.0f ||
      !isfinite(windSpeedKmh) || windSpeedKmh < 0.0f || windSpeedKmh > 300.0f ||
      !isfinite(rainAmountMm) || rainAmountMm < 0.0f || rainAmountMm > 1000.0f ||
      !isfinite(rainChancePct) || rainChancePct < 0.0f || rainChancePct > 100.0f)
    return false;

  int64_t fetchedEpoch;
  if (!parseIsoTimestamp(fetchedAt, fetchedEpoch)) return false;

  memset(&result, 0, sizeof(result));
  result.valid = true;
  strlcpy(result.locationCode, locationCode, sizeof(result.locationCode));
  strlcpy(result.fetchedAt, fetchedAt, sizeof(result.fetchedAt));
  result.fetchedEpoch = fetchedEpoch;
  result.temperatureC = temperatureC;
  result.humidityPct = humidityPct;
  result.windSpeedKmh = windSpeedKmh;
  result.rainAmountMm = rainAmountMm;
  result.rainChancePct = rainChancePct;
  return true;
}

void restoreBmkgSnapshot() {
  if (!weatherStorageReady) return;
  String cachedPayload = weatherPreferences.getString("snapshot", "");
  if (cachedPayload.length() == 0) return;

  BmkgSnapshot restored;
  if (parseBmkgPayload(reinterpret_cast<const uint8_t*>(cachedPayload.c_str()),
                       cachedPayload.length(), restored)) {
    bmkgSnapshot = restored;
    Serial.printf("[BMKG] Restored cached snapshot for %s (%s)\n",
                  bmkgSnapshot.locationCode, bmkgSnapshot.fetchedAt);
  } else {
    Serial.println("[BMKG] Stored snapshot is invalid; ignoring it");
  }
}

const char* bmkgFreshness(int64_t& ageSeconds) {
  ageSeconds = -1;
  if (!bmkgSnapshot.valid) return "unavailable";
  time_t now = time(NULL);
  if (now < 1577836800 || static_cast<int64_t>(now) + 300 < bmkgSnapshot.fetchedEpoch)
    return "unknown";
  ageSeconds = static_cast<int64_t>(now) - bmkgSnapshot.fetchedEpoch;
  return ageSeconds >= BMKG_STALE_AFTER_SECONDS ? "stale" : "fresh";
}

void appendBmkgTelemetry(String& json) {
  if (!bmkgSnapshot.valid) return;
  int64_t ageSeconds;
  const char* freshness = bmkgFreshness(ageSeconds);
  json += ",\"bmkg_weather\":{";
  json += "\"source\":\"BMKG\",";
  json += "\"location_code\":\"" + String(bmkgSnapshot.locationCode) + "\",";
  json += "\"fetched_at\":\"" + String(bmkgSnapshot.fetchedAt) + "\",";
  json += "\"freshness\":\"" + String(freshness) + "\",";
  if (ageSeconds >= 0) json += "\"age_seconds\":" + String(static_cast<long>(ageSeconds)) + ",";
  json += "\"temperature_c\":" + String(bmkgSnapshot.temperatureC, 2) + ",";
  json += "\"humidity_pct\":" + String(bmkgSnapshot.humidityPct, 2) + ",";
  json += "\"wind_speed_kmh\":" + String(bmkgSnapshot.windSpeedKmh, 2) + ",";
  json += "\"rain_amount_mm\":" + String(bmkgSnapshot.rainAmountMm, 2) + ",";
  json += "\"rain_chance_pct\":" + String(bmkgSnapshot.rainChancePct, 2);
  json += "}";
}

void enrichTelemetry(String& json) {
  if (!bmkgSnapshot.valid || !json.endsWith("}") ||
      json.indexOf("\"bmkg_weather\"") >= 0) return;
  json.remove(json.length() - 1);
  appendBmkgTelemetry(json);
  json += "}";
}

void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  if (strcmp(topic, BMKG_TOPIC) != 0) return;

  BmkgSnapshot incoming;
  if (!parseBmkgPayload(payload, length, incoming)) {
    Serial.println("[BMKG] Rejected malformed or unsupported weather snapshot");
    return;
  }

  bmkgSnapshot = incoming;
  if (weatherStorageReady) {
    char payloadText[BMKG_MAX_PAYLOAD_SIZE + 1];
    memcpy(payloadText, payload, length);
    payloadText[length] = '\0';
    if (weatherPreferences.putString("snapshot", payloadText) == 0) {
      Serial.println("[BMKG] NVS snapshot write failed");
    }
  }
  Serial.printf("[BMKG] Cached weather snapshot for %s (%s)\n",
                bmkgSnapshot.locationCode, bmkgSnapshot.fetchedAt);
}

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
        enrichTelemetry(cleanJson);
        
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
      configTime(0, 0, "pool.ntp.org", "time.google.com");
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
    if (!mqttClient.subscribe(BMKG_TOPIC, 1)) {
      Serial.println("[BMKG] Failed to subscribe to weather snapshot topic");
    }
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

  weatherStorageReady = weatherPreferences.begin("bmkg", false);
  if (weatherStorageReady) {
    restoreBmkgSnapshot();
  } else {
    Serial.println("[BMKG] NVS unavailable; weather cache is RAM-only");
  }

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
    configTime(0, 0, "pool.ntp.org", "time.google.com");
  }

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setKeepAlive(15); 
  // Set buffer size MQTT lebih besar agar tidak mudah gagal
  mqttClient.setBufferSize(1024);

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
    enrichTelemetry(json);

    processData(data.nodeID, json);
  }
}