#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <time.h>
#include <esp_sntp.h>
#include <esp_task_wdt.h>   // Untuk melepas IDLE0 dari WDT saat TLS handshake
#include <Preferences.h>    // Penyimpanan permanen NVS Flash untuk mode dan WiFi
#include "secret.h"

Preferences prefs;

// Forward Declarations
void ensureMqttConnection();
void publishActuatorMqtt();
void publishIrrigationScoreMqtt();
void publishScheduleMqtt();
void applyNewWiFi(const String& newSsid, const String& newPass);
void saveAgriSettingsNVS();
void loadAgriSettingsNVS();
void startIrigasiTerjadwal(int durasiDetik);
void evaluasiJadwalPenyiraman();
void startFertigasiSiklus3Tahap(float volNutrisi, float customPreWet = 0.0f, float customPostFlush = 0.0f, bool karhutla = false);

// =====================================================
// WIFI CONFIG (Dinamis dari NVS Flash, default dari secret.h)
// =====================================================
String currentSsid = "";
String currentPass = "";

#ifndef WIFI_SSID
#endif
#ifndef WIFI_PASS
#endif
#ifndef MQTT_SERVER
#endif
#ifndef MQTT_PORT_VALUE
#define MQTT_PORT_VALUE 8883
#endif
#ifndef MQTT_USERNAME
#endif
#ifndef MQTT_PASSWORD
#endif

// =========================================================================
// SISTEM GATEWAY SMART AGRICULTURE & MITIGASI KARHUTLA GAMBUT (TERANA / NEXAGROW)
// ARSITEKTUR: ESP32 DUAL-CORE FREERTOS
// Core 0: Protocol, Networking (WiFi, MQTT TLS, Buffering), Serial2 UART
// Core 1: Hardware I/O, Local pH Sensor, Water Flow FSM, TFT Display Rendering
// =========================================================================

// =====================================================
// HARDWARE SERIAL2 (Dari ESP8266 Receiver)
// =====================================================
// Pin GPIO 16 (RX2) terhubung ke TX ESP8266 Receiver
// Pin GPIO 17 (TX2) terhubung ke RX ESP8266 Receiver
#define UART_RX_PIN 3
#define UART_TX_PIN 1
#define UART_BAUD   115200

// =====================================================
// PIN HARDWARE SENSOR & AKTUATOR
// =====================================================
#define PIN_PH_SENSOR        36  // GPIO 36 / VP (ADC1_CH0 - Input Analog Sensor pH)
#define PIN_FLOW_SENSOR      35  // GPIO 27 (Sensor Water Flow YF-S201 Hall-Effect Interrupt)
#define PIN_RELAY_IRIGASI    33  // GPIO 33 (Relay 1 - Solenoid Valve Irigasi Air Bersih)
#define PIN_RELAY_FERTIGASI  32  // GPIO 32 (Relay 2 - Solenoid Valve Fertigasi / Amelioran)

// =====================================================
// PIN RELAY LAMPU INDIKATOR SISTEM (POWER/SISTEM & WIFI)
// =====================================================
#define PIN_RELAY_LAMP_SYSTEM 26 // GPIO 26 / D26: Relay Lampu Indikator Sistem Menyala (Selalu ON saat ESP32 aktif)
#define PIN_RELAY_LAMP_ONLINE 25 // GPIO 25 / D25: Relay Lampu Koneksi (ON = Online/WiFi Terkoneksi, OFF = Offline)

// Logika Modul Relay (Aktif LOW)
#define RELAY_ACTIVE_LEVEL   LOW
#define RELAY_INACTIVE_LEVEL HIGH

// Kalibrasi Sensor Waterflow (YF-S201: ~450 pulsa / Liter)
#define FLOW_PULSES_PER_LITER 450.0f

// =====================================================
// KALIBRASI SENSOR pH ANALOG (LOKAL ESP32 ADC)
// =====================================================
// Menggunakan ADC1 GPIO 34 (12-bit: 0 - 4095)
// pH = slope * voltage + intercept
#define PH_SAMPLES       15       // Jumlah sampel averaging
#define PH_ADC_MAX       4095.0f  // Resolusi 12-bit ESP32
#define PH_VREF          3.3f     // Tegangan referensi ADC ESP32
static const float phNeutralVoltage = 2.50f;   // Tegangan saat pH 7.0
static const float phAcidVoltage    = 2.032f;  // Tegangan saat pH 4.0

// =====================================================
// MQTT CONFIG (Dari secret.h - HiveMQ Cloud TLS)
// =====================================================
const char* mqtt_server = MQTT_SERVER;
const int   mqtt_port   = MQTT_PORT_VALUE;
const char* mqtt_user   = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_client_id = "ESP32_Gateway_Nexa";

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

// =====================================================
// TFT DISPLAY INSTANCE (ILI9488 SPI 320x480 Landscape)
// =====================================================
TFT_eSPI tft = TFT_eSPI();

// Palet Warna Modern (RGB565)
#define COLOR_BG          0x0841  // Dark Navy Slate
#define COLOR_CARD_BG     0x10A2  // Dark Modern Card Background
#define COLOR_HEADER      0x0185  // Deep Slate Top Bar
#define COLOR_TEXT        0xFFFF  // Putih
#define COLOR_TEXT_MUTED  0x9CD3  // Abu-abu Metalik
#define COLOR_ACCENT      0x07E0  // Hijau Neon
#define COLOR_CYAN        0x07FF  // Cyan Cerah
#define COLOR_ORANGE      0xFD20  // Orange Gambut
#define COLOR_RED         0xF800  // Merah Alarm / Offline
#define COLOR_YELLOW      0xFFE0  // Kuning
#define COLOR_BLUE        0x3CD9  // Biru Muda
#define COLOR_BORDER_N1   0x03E8  // Border Card 1
#define COLOR_BORDER_N2   0x0279  // Border Card 2
#define COLOR_BORDER_N3   0xFD20  // Border Card 3
#define COLOR_BORDER_WTR  0x05F5  // Border Card 4

// =====================================================
// DATA STRUKTUR SENSOR (THREAD-SAFE DENGAN MUTEX)
// =====================================================
struct NodeData {
  float temperature = NAN;
  float humidity = NAN;
  float soil_moisture = NAN;
  float battery = NAN;
  uint32_t counter = 0;
  unsigned long lastSeen = 0;
  unsigned long prevSeen = 0;       // Waktu paket sebelumnya (untuk estimasi interval sleep)
  uint16_t sleepDurationSec = 30;   // Estimasi durasi deep sleep aktif node (default 30s)
  bool active = false;
};

// Data Khusus Node 3 (Suhu Tanah Gambut DS18B20 & BMKG FDRS)
struct Node3Data {
  float soil_temp      = NAN;   // MURNI: Suhu Tanah Gambut °C (DS18B20 probe)
  float battery        = NAN;   // Voltase Baterai Node 3 (A0)
  float temperature    = NAN;   // Suhu Ambient Wilayah Gambut °C
  float humidity       = NAN;   // RH Ambient Wilayah Gambut %
  float ffmc           = NAN;   // Fine Fuel Moisture Code
  float dmc            = NAN;   // Duff Moisture Code
  float dc             = NAN;   // Drought Code (Akumulasi pengeringan gambut dalam)
  float isi            = NAN;   // Initial Spread Index
  float bui            = NAN;   // Build-Up Index
  float fwi_score      = NAN;   // Fire Weather Index final BMKG
  uint8_t fwi_level    = 0;     // 0=RENDAH 1=SEDANG 2=TINGGI 3=SANGAT TINGGI 4=DARURAT
  uint32_t counter     = 0;
  unsigned long lastSeen = 0;
  unsigned long prevSeen = 0;       // Waktu paket sebelumnya (untuk estimasi interval sleep)
  uint16_t sleepDurationSec = 30;   // Estimasi durasi deep sleep aktif node (default 30s)
  bool active          = false;
};

// Kategori bahaya BMKG-FDRS
static const char* BMKG_LEVEL_STR[] = {
  "RENDAH",        // 0 🔵 Biru
  "SEDANG",        // 1 🟢 Hijau
  "TINGGI",        // 2 🟡 Kuning
  "SANGAT TINGGI", // 3 🔴 Merah
  "DARURAT"        // 4 🚨 Smoldering fire terdeteksi (DS18B20 ≥ 50°C)
};

static const uint16_t BMKG_LEVEL_COLOR[] = {
  0x03DF, // RENDAH        = Biru    🔵
  0x07E0, // SEDANG        = Hijau   🟢
  0xFFE0, // TINGGI        = Kuning  🟡
  0xF800, // SANGAT TINGGI = Merah   🔴
  0xF81F  // DARURAT       = Magenta 🚨
};

// =====================================================
// HELPER PERHITUNGAN PERSENTASE BATERAI LI-PO (1S 3.7V)
// 3.00V = 0% (Cutoff Empty), 4.20V = 100% (Full Charged)
// =====================================================
inline int hitungPersenBaterai(float voltage) {
  if (isnan(voltage) || voltage <= 3.00f) return 0;
  if (voltage >= 4.20f) return 100;
  float pct = ((voltage - 3.00f) / (4.20f - 3.00f)) * 100.0f;
  return (int)constrain(pct, 0.0f, 100.0f);
}

// Variabel Global Sensor
NodeData node1;
NodeData node2;
Node3Data node3;
float gatewayPh = NAN;
unsigned long lastPhReadTime = 0;
unsigned long lastPhMqttPub = 0;
unsigned long totalPacketsReceived = 0;

// Mutex FreeRTOS untuk sinkronisasi data antar Core 0 & Core 1
SemaphoreHandle_t xDataMutex = NULL;
// Mutex FreeRTOS (Recursive) untuk keamanan transmisi TLS MQTT antar Core 0 & Core 1
SemaphoreHandle_t xMqttMutex = NULL;
// Mutex FreeRTOS untuk antrean data buffer offline (Thread-safe antar Core 0 & Core 1)
SemaphoreHandle_t xBufferMutex = NULL;
// Mutex FreeRTOS untuk akses penyimpanan Flash NVS (Preferences) thread-safe
SemaphoreHandle_t xPrefsMutex = NULL;

bool safeMqttPublish(const char* topic, const char* payload, bool retained = false) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!mqttClient.connected()) return false;
  if (xMqttMutex != NULL && xSemaphoreTakeRecursive(xMqttMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool ok = false;
    if (mqttClient.connected()) {
      ok = mqttClient.publish(topic, payload, retained);
      if (ok) {
        Serial.printf("[TX -> WEB] %s => %s\n", topic, payload);
      }
    }
    xSemaphoreGiveRecursive(xMqttMutex);
    return ok;
  }
  return false;
}

// =====================================================
// STATUS AKTUATOR & FINITE STATE MACHINE (FSM)
// =====================================================
String modeSistem = "MANUAL"; // Dimuat dari Flash NVS pada setup()
bool statusPompa = false;
bool statusIrigasi = false;
bool statusFertigasi = false;
bool statusLampuSystem = true;
bool statusLampuOnline = false;

volatile uint32_t totalFlowPulses = 0;
volatile uint32_t sessionFlowPulses = 0;
float flowRateLpm = 0.0f;
float totalWaterLiters = 0.0f;
float sessionWaterLiters = 0.0f;
uint32_t lastSnapPulses = 0;
unsigned long lastFlowRateCalcTime = 0;

// Auto-deteksi sesi aliran baru (reset otomatis saat aliran baru dimulai setelah idle)
bool flowIsIdle = true;                // Status: true = tidak ada aliran air
unsigned long flowIdleSince = 0;       // Waktu mulai idle (ms)
const unsigned long FLOW_IDLE_THRESHOLD = 5000UL; // 5 detik tanpa aliran = dianggap berhenti

// Tracking perubahan nilai untuk optimasi penulisan NVS Flash (hemat siklus tulis)
float lastSavedPh = NAN;
float lastSavedWater = 0.0f;

void IRAM_ATTR flowSensorISR() {
  totalFlowPulses++;
  sessionFlowPulses++;
}

enum ActuatorState {
  STATE_IDLE,
  STATE_IRIGASI_ACTIVE,
  STATE_INTER_DELAY,
  STATE_FERTIGASI_ACTIVE,
  STATE_POST_FLUSH_DELAY,
  STATE_POST_FLUSH_ACTIVE
};

ActuatorState actuatorState = STATE_IDLE;
float targetLiterIrigasi = 0.0f;
float targetLiterFertigasi = 0.0f;
float targetLiterPostFlush = 0.0f;
unsigned long sessionStartTime = 0;
unsigned long interDelayStartTime = 0;
unsigned long postFlushDelayStartTime = 0;
bool queueFertigasiPending = false;
float queuedFertigasiTarget = 0.0f;
bool is3PhaseFertCycle = false;
bool isKarhutlaCycle = false;

unsigned long lastAutoCycleTime = 0;
const unsigned long COOLDOWN_AGRI_MS = 15UL * 60UL * 1000UL;
const unsigned long COOLDOWN_KARHUTLA_MS = 5UL * 60UL * 1000UL;

// =========================================================================
// PARAMETER LOGIKA KONTROL FERTIGASI CERDAS (HYBRID HORTIKULTURA & GAMBUT)
// =========================================================================
int intervalHariFertigasi       = 2;     // Siklus pemupukan setiap N hari sekali (Default: 2 hari)
int lastFertigasiDay            = -1;    // Hari terakhir (1-31) fertigasi dieksekusi
int lastFertigasiMonth          = -1;    // Bulan terakhir (1-12)
int lastFertigasiYear           = -1;    // Tahun terakhir
unsigned long lastFertigasiTimestamp = 0; // Millis tracking untuk fallback saat waktu NTP offline
const unsigned long COOLDOWN_FERTIGASI_OFFLINE_MS = 6UL * 3600UL * 1000UL; // Minimal 6 jam jeda jika NTP offline

// Batas Ceiling Maksimum Dosis (Anti-Overdose Safety Guard)
const float MAX_FERTIGASI_LITER_NORMAL = 2.5f; // Maksimal 2.5 L nutrisi pada kebun biasa
const float MAX_FERTIGASI_LITER_GAMBUT = 3.5f; // Maksimal 3.5 L amelioran pada gambut

// =========================================================================
// PARAMETER LOGIKA IRIGASI HORTIKULTURA (LAHAN TIDAK GAMBUT)
// Berdasarkan Referensi Formula Arduino Code Nexa & Sistem Teranova
// =========================================================================
// 1. Ambang Batas Kelembapan Tanah (%)
float ambangAtas       = 75.0f; // Batas atas kelembapan optimal
float ambangBawah      = 45.0f; // Batas bawah mulai waspada
float ambangKritis     = 35.0f; // Batas darurat kekeringan kritis

// 1b. Ambang Batas Suhu & Kelembapan Udara Lingkungan (°C & %)
float ambangTempLow    = 22.0f; // Batas bawah suhu ideal
float ambangTempHigh   = 34.0f; // Batas atas suhu (mulai stres panas)
float ambangHumLow     = 60.0f; // Batas bawah kelembapan udara (mulai kering)
float ambangHumHigh    = 85.0f; // Batas atas kelembapan udara

// 2. Fase Tanaman (Vegetatif vs Generatif)
bool faseVegetatif     = true;  // true = Vegetatif, false = Generatif
float ambangSkorAktif  = 55.0f; // Ambang skor untuk menyalakan irigasi (Vegetatif: 55, Generatif: 60)
float ambangSkorBawah  = 25.0f; // Ambang skor bawah histeresis (Vegetatif: 25, Generatif: 35)

// 3. Penjadwalan Waktu (Scheduling)
bool jadwalPenyiramanAktif     = true;
int jadwalJam                  = 6;     // Pukul 06:00
int jadwalMenit                = 0;
int jadwalPenyiramanDurasiDetik = 10;
int lastWateredDay             = -1;    // Tracker hari terakhir disiram (agar berulang setiap hari baru)
int lastWateredHour            = -1;
int lastWateredMinute          = -1;
int targetDurasiIrigasiDetik   = 0;     // Kontrol durasi irigasi terjadwal (detik)

// 4. Cuaca & Peluang Hujan BMKG (%)
float peluangHujan             = 0.0f;

// 5. Metrik Scoring Real-Time (Kalkulasi Otomatis dari 2 Node)
float vpdAgri                  = 0.0f;  // Vapor Pressure Deficit (kPa)
float skorTanah                = 0.0f;  // Skor defisit tanah (0 - 50)
float skorVpd                  = 0.0f;  // Skor tuntutan VPD (0 - 30)
float skorInteraksi            = 0.0f;  // Skor sinergi tanah & VPD (0 - 30)
float skorHujan                = 0.0f;  // Pengurang skor hujan (0 - 40)
float skorTotal                = 0.0f;  // Total skor kebutuhan air
bool siklusIrigasiAktif        = false; // Status siklus histeresis
float agriSoilRep              = NAN;   // Kelembapan tanah representatif 2 node
float agriSoilMin              = NAN;   // Kelembapan tanah terendah (kritis)
float agriTemp                 = NAN;   // Suhu gabungan Node 1 & 2
float agriRh                   = NAN;   // RH gabungan Node 1 & 2

// =====================================================
// DATA OFFLINE BUFFER (Saat Koneksi Terputus)
// =====================================================
#define BUFFER_SIZE 30
String dataBuffer[BUFFER_SIZE];
String topicBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferRead = 0;
int bufferCount = 0;

void bufferAdd(String topic, String json) {
  if (xBufferMutex != NULL && xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
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
    xSemaphoreGive(xBufferMutex);
  }
}

void flushBuffer() {
  if (WiFi.status() != WL_CONNECTED || !mqttClient.connected() || bufferCount == 0) return;
  if (xBufferMutex != NULL && xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    int initialCount = bufferCount;
    for (int i = 0; i < initialCount; ++i) {
      String p = dataBuffer[bufferRead];
      String t = topicBuffer[bufferRead];
      if (p.length() > 0 && t.length() > 0) {
        if (safeMqttPublish(t.c_str(), p.c_str(), false)) {
          dataBuffer[bufferRead] = "";
          topicBuffer[bufferRead] = "";
          bufferRead = (bufferRead + 1) % BUFFER_SIZE;
          bufferCount--;
        } else {
          break;
        }
      } else {
        bufferRead = (bufferRead + 1) % BUFFER_SIZE;
        bufferCount--;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    xSemaphoreGive(xBufferMutex);
  }
}

// =====================================================
// PEMBACAAN SENSOR pH LOKAL PADA ESP32 (ADC GPIO 34)
// =====================================================
float bacaSensorPhLokal() {
  long totalRaw = 0;
  for (int i = 0; i < PH_SAMPLES; i++) {
    totalRaw += analogRead(PIN_PH_SENSOR);
    delayMicroseconds(500);
  }
  float avgRaw = (float)totalRaw / (float)PH_SAMPLES;
  float voltage = avgRaw * (PH_VREF / PH_ADC_MAX);

  // Rumus Linear Interpolasi 2 Titik Kalibrasi
  float slope = (7.0f - 4.0f) / (phNeutralVoltage - phAcidVoltage);
  float intercept = 7.0f - slope * phNeutralVoltage;
  float ph = slope * voltage + intercept;

  return constrain(ph, 0.0f, 14.0f);
}

// ======================================================================
// BMKG-FDRS FIRE WEATHER INDEX ENGINE (ESP32 GATEWAY)
// Van Wagner 1987 + Adaptasi Lapisan Gambut Indonesia (SPARTAN BMKG)
// ======================================================================
static float gwFFMC = 85.0f;
static float gwDMC  = 30.0f;
static float gwDC   = 200.0f;
float externalWindKmh = 0.0f;

float hitungFFMC(float T, float RH, float prevFFMC) {
  if (isnan(T) || isnan(RH)) return prevFFMC;
  float Mo = 147.2f * (101.0f - prevFFMC) / (59.5f + prevFFMC);
  float Ed = 0.942f * powf(RH, 0.679f) + 11.0f * expf((RH - 100.0f) / 10.0f) + 0.18f * (21.1f - T) * (1.0f - expf(-0.115f * RH));
  float Ew = 0.618f * powf(RH, 0.753f) + 10.0f * expf((RH - 100.0f) / 10.0f) + 0.18f * (21.1f - T) * (1.0f - expf(-0.115f * RH));
  float m;
  if (Mo > Ed) {
    float kd = (0.424f * (1.0f - powf(RH / 100.0f, 1.7f)) + 0.0694f * (1.0f - powf(RH / 100.0f, 8.0f))) * 0.581f * expf(0.0365f * T);
    m = Ed + (Mo - Ed) * powf(10.0f, -kd);
  } else if (Mo < Ew) {
    float kw = (0.424f * (1.0f - powf((100.0f - RH) / 100.0f, 1.7f)) + 0.0694f * (1.0f - powf((100.0f - RH) / 100.0f, 8.0f))) * 0.581f * expf(0.0365f * T);
    m = Ew - (Ew - Mo) * powf(10.0f, -kw);
  } else {
    m = Mo;
  }
  float ffmc = 59.5f * (250.0f - m) / (147.2f + m);
  return constrain(ffmc, 0.0f, 101.0f);
}

float hitungDMC(float T, float RH, float prevDMC) {
  if (isnan(T) || isnan(RH)) return prevDMC;
  float El = 0.0f;
  if (T >= -1.1f) {
    float K = 1.894f * (T + 1.1f) * (100.0f - RH) * 0.0001f;
    El = 1.1f * K * 24.0f;
  }
  float newDMC = prevDMC + El;
  return max(newDMC, 0.0f);
}

float hitungDC(float T, float soilTemp, float prevDC) {
  float Vd = 0.0f;
  if (!isnan(T) && T >= -2.8f) {
    Vd = (0.36f * (T + 2.8f) + 9.0f) / 2.0f;
  }
  float newDC = prevDC + (0.5f * Vd);

  // KOREKSI UTAMA DARI SENSOR SUHU TANAH GAMBUT DS18B20:
  // Kenaikan suhu di kedalaman gambut mengindikasikan akumulasi panas & kekeringan ekstrem
  if (!isnan(soilTemp)) {
    if (soilTemp >= 35.0f) newDC += 10.0f;
    if (soilTemp >= 40.0f) newDC += 25.0f; // Pemanasan signifikan
    if (soilTemp >= 45.0f) newDC += 45.0f; // Indikasi bara smoldering
  }
  return max(newDC, 0.0f);
}

float hitungISI(float ffmc, float windKmh) {
  if (isnan(ffmc)) return 0.0f;
  float fw = expf(0.05039f * windKmh);
  float m = 147.2f * (101.0f - ffmc) / (59.5f + ffmc);
  float fF = 91.9f * expf(-0.1386f * m) * (1.0f + powf(m, 5.31f) / 49300000.0f);
  return max(0.208f * fw * fF, 0.0f);
}

float hitungBUI(float dmc, float dc) {
  if (isnan(dmc) || isnan(dc)) return 0.0f;
  if (dmc == 0.0f && dc == 0.0f) return 0.0f;
  float bui;
  if (dmc <= (0.4f * dc)) {
    bui = 0.8f * dmc * dc / (dmc + 0.4f * dc);
  } else {
    bui = dmc - (1.0f - 0.8f * dc / (dmc + 0.4f * dc)) * (0.92f + powf(0.0114f * dmc, 1.7f));
  }
  return max(bui, 0.0f);
}

float hitungFWI(float isi, float bui) {
  if (isnan(isi) || isnan(bui)) return 0.0f;
  float fD = (bui <= 80.0f) ? (0.626f * powf(bui, 0.809f) + 2.0f) : (1000.0f / (25.0f + 108.64f * expf(-0.023f * bui)));
  float B = 0.1f * isi * fD;
  float fwi = (B > 1.0f) ? expf(2.72f * powf(0.434f * logf(B), 0.647f)) : B;
  return max(fwi, 0.0f);
}

uint8_t tentukanFireLevelGateway(float fwi, float soilTemp) {
  // Override Level 4 (DARURAT): Terdeteksi bara api bawah permukaan gambut (DS18B20 ≥ 50°C)
  if (!isnan(soilTemp) && soilTemp >= 50.0f) return 4;
  if (isnan(fwi)) return 0;
  // Klasifikasi BMKG SPARTAN resmi
  if (fwi > 32.0f) return 3; // SANGAT TINGGI 🔴
  if (fwi > 17.0f) return 2; // TINGGI        🟡
  if (fwi >  8.0f) return 1; // SEDANG        🟢
  return 0;                   // RENDAH        🔵
}

// =====================================================
// PUBLISH KE MQTT BROKER
// =====================================================
void publishNode3Mqtt(const Node3Data& n3) {
  const String topicNode = "sproutai/sensor/node/3";
  const String topicData = "sproutai/sensor/data";
  const String topicFWI  = "sproutai/karhutla/fwi";
  const String topicAlrt = "sproutai/karhutla/alert";

  uint8_t lvl = n3.fwi_level < 5 ? n3.fwi_level : 4;
  const char* levelStr = BMKG_LEVEL_STR[lvl];

  char json[384];
  snprintf(json, sizeof(json),
    "{"
    "\"node_id\":3,"
    "\"soil_temp\":%.2f,"        // Suhu Tanah Gambut DS18B20
    "\"battery\":%.2f,"          // Voltase Baterai Node 3
    "\"battery_pct\":%d,"        // Persentase Baterai Node 3 (Li-Po 1S)
    "\"temperature\":%.1f,"      // Suhu Ambient
    "\"humidity\":%.1f,"         // RH Ambient
    "\"ffmc\":%.2f,"
    "\"dmc\":%.2f,"
    "\"dc\":%.2f,"
    "\"isi\":%.2f,"
    "\"bui\":%.2f,"
    "\"fwi\":%.2f,"
    "\"fwi_level\":%d,"
    "\"fwi_status\":\"%s\","
    "\"counter\":%lu"
    "}",
    isnan(n3.soil_temp)   ? 0.0f : n3.soil_temp,
    isnan(n3.battery)     ? 0.0f : n3.battery,
    hitungPersenBaterai(n3.battery),
    isnan(n3.temperature) ? 0.0f : n3.temperature,
    isnan(n3.humidity)    ? 0.0f : n3.humidity,
    isnan(n3.ffmc)        ? 0.0f : n3.ffmc,
    isnan(n3.dmc)         ? 0.0f : n3.dmc,
    isnan(n3.dc)          ? 0.0f : n3.dc,
    isnan(n3.isi)         ? 0.0f : n3.isi,
    isnan(n3.bui)         ? 0.0f : n3.bui,
    isnan(n3.fwi_score)   ? 0.0f : n3.fwi_score,
    lvl,
    levelStr,
    (unsigned long)n3.counter
  );

  if (mqttClient.connected()) {
    safeMqttPublish(topicNode.c_str(), json, false);
    safeMqttPublish(topicData.c_str(), json, false);
    safeMqttPublish(topicFWI.c_str(),  json, true);

    if (n3.fwi_level >= 2) {
      char alert[200];
      snprintf(alert, sizeof(alert),
        "{\"fwi_level\":%d,\"fwi_status\":\"%s\",\"fwi\":%.2f,\"soil_temp\":%.1f,\"smoldering\":%s}",
        lvl, levelStr,
        isnan(n3.fwi_score) ? 0.0f : n3.fwi_score,
        isnan(n3.soil_temp) ? 0.0f : n3.soil_temp,
        (n3.fwi_level >= 4 || n3.soil_temp >= 45.0f) ? "true" : "false");
      safeMqttPublish(topicAlrt.c_str(), alert, false);
    }
  } else {
    bufferAdd(topicNode, String(json));
    bufferAdd(topicFWI,  String(json));
  }
}

void publishPhMqtt(float ph) {
  if (isnan(ph)) return;
  char json[96];
  snprintf(json, sizeof(json), "{\"node_id\":0,\"ph\":%.2f}", ph);
  if (mqttClient.connected()) {
    safeMqttPublish("sproutai/sensor/ph", json, false);
    safeMqttPublish("sproutai/sensor/data", json, false);
  } else {
    bufferAdd("sproutai/sensor/ph", String(json));
  }
}

void processData(int nodeId, const String& jsonPayload) {
  String topic = "sproutai/sensor/node/" + String(nodeId);
  String topicData = "sproutai/sensor/data";

  if (mqttClient.connected()) {
    safeMqttPublish(topic.c_str(), jsonPayload.c_str(), true); // Retained agar web langsung tampilkan data terakhir
    safeMqttPublish(topicData.c_str(), jsonPayload.c_str(), false);
  } else {
    bufferAdd(topic, jsonPayload);
    bufferAdd(topicData, jsonPayload);
  }
}

void publishActuatorMqtt() {
  if (!mqttClient.connected()) return;

  const char* activeValve = (actuatorState == STATE_IRIGASI_ACTIVE) ? (is3PhaseFertCycle ? "PRE_WET" : "IRIGASI") :
                            (actuatorState == STATE_FERTIGASI_ACTIVE) ? "FERTIGASI" :
                            (actuatorState == STATE_POST_FLUSH_ACTIVE) ? "POST_FLUSH" : "IDLE";

  char flowJson[180];
  snprintf(flowJson, sizeof(flowJson),
    "{\"flow_rate\":%.2f,\"session_liters\":%.2f,\"total_liters\":%.2f,\"active_valve\":\"%s\"}",
    flowRateLpm, sessionWaterLiters, totalWaterLiters, activeValve);
  safeMqttPublish("sproutai/sensor/flow", flowJson, false);

  bool isAuto = modeSistem.equalsIgnoreCase("OTOMATIS") || modeSistem.equalsIgnoreCase("AUTO");
  bool isOnline = (WiFi.status() == WL_CONNECTED);

  char actJson[420];
  snprintf(actJson, sizeof(actJson),
    "{\"irigasi\":%s,\"fertigasi\":%s,\"pompa\":%s,\"mode\":\"%s\","
    "\"lampu_sistem\":true,\"lampu_online\":%s,"
    "\"target_liters\":%.2f,\"current_liters\":%.2f,\"total_liters\":%.2f,\"karhutla_mode\":%s,"
    "\"siklus_3tahap\":%s,\"active_valve\":\"%s\"}",
    statusIrigasi ? "true" : "false",
    statusFertigasi ? "true" : "false",
    statusPompa ? "true" : "false",
    modeSistem.c_str(),
    isOnline ? "true" : "false",
    (actuatorState == STATE_IRIGASI_ACTIVE) ? targetLiterIrigasi :
    (actuatorState == STATE_FERTIGASI_ACTIVE) ? targetLiterFertigasi :
    (actuatorState == STATE_POST_FLUSH_ACTIVE) ? targetLiterPostFlush : 0.0f,
    sessionWaterLiters,
    totalWaterLiters,
    isKarhutlaCycle ? "true" : "false",
    is3PhaseFertCycle ? "true" : "false",
    activeValve);
  // Retained = true agar broker HiveMQ menyimpan status terakhir untuk browser baru/refresh
  safeMqttPublish("sproutai/actuator/status", actJson, true);
  safeMqttPublish("sproutai/pompa/status", statusPompa ? "ON" : "OFF", true);
  safeMqttPublish("sproutai/mode/status", modeSistem.c_str(), true);
  safeMqttPublish("sproutai/lampu/system/status", "ON", true);
  safeMqttPublish("sproutai/lampu/online/status", isOnline ? "ON" : "OFF", true);
}

// =====================================================
// KONTROL LAMPU INDIKATOR SISTEM & KONEKSI WIFI (D26 & D25)
// =====================================================
void updateIndicatorLamps(bool force = false) {
  static int lastWifiVal = -1;

  if (force) {
    lastWifiVal = -1;
    // Lampu Indikator Sistem Menyala (Pin D26): Langsung ON saat ESP32 aktif
    pinMode(PIN_RELAY_LAMP_SYSTEM, OUTPUT);
    digitalWrite(PIN_RELAY_LAMP_SYSTEM, RELAY_ACTIVE_LEVEL);
    statusLampuSystem = true;
    Serial.println(F("[LAMP RELAY] D26 System Lamp: ON (ESP32 Powered ON)"));
  }

  // Lampu Indikator Koneksi WiFi (Pin D25): ON jika WiFi Terkoneksi, OFF jika Offline
  bool isOnline = (WiFi.status() == WL_CONNECTED);
  int currentWifiVal = isOnline ? 1 : 0;
  if (currentWifiVal != lastWifiVal) {
    lastWifiVal = currentWifiVal;
    statusLampuOnline = isOnline;
    digitalWrite(PIN_RELAY_LAMP_ONLINE, isOnline ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    Serial.printf("[LAMP RELAY] D25 WiFi Online Lamp: %s (Status: %s)\n",
                  isOnline ? "ON" : "OFF", isOnline ? "CONNECTED" : "DISCONNECTED");
  }
}

// =====================================================
// KONTROL HARDWARE AKTUATOR & LOGIKA CLOSED-LOOP
// =====================================================
void initActuatorHardware() {
  pinMode(PIN_RELAY_IRIGASI, OUTPUT);
  digitalWrite(PIN_RELAY_IRIGASI, RELAY_INACTIVE_LEVEL);

  pinMode(PIN_RELAY_FERTIGASI, OUTPUT);
  digitalWrite(PIN_RELAY_FERTIGASI, RELAY_INACTIVE_LEVEL);

  // Inisialisasi Relay Lampu Indikator Sistem (D26) & WiFi Online (D25)
  // Pin D26 langsung dinyalakan sebagai penanda ESP32 sudah aktif/menyala
  pinMode(PIN_RELAY_LAMP_SYSTEM, OUTPUT);
  digitalWrite(PIN_RELAY_LAMP_SYSTEM, RELAY_ACTIVE_LEVEL);
  pinMode(PIN_RELAY_LAMP_ONLINE, OUTPUT);
  digitalWrite(PIN_RELAY_LAMP_ONLINE, RELAY_INACTIVE_LEVEL);
  updateIndicatorLamps(true);

  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), flowSensorISR, RISING);

  pinMode(PIN_PH_SENSOR, INPUT);
  analogReadResolution(12); // Pastikan 12-bit ADC
}

void updateFlowSensorMetrics() {
  unsigned long now = millis();
  if (now - lastFlowRateCalcTime >= 1000UL) {
    unsigned long dt = now - lastFlowRateCalcTime;
    lastFlowRateCalcTime = now;

    noInterrupts();
    uint32_t curTotal = totalFlowPulses;
    uint32_t curSession = sessionFlowPulses;
    interrupts();

    uint32_t deltaPulses = curTotal - lastSnapPulses;
    lastSnapPulses = curTotal;

    flowRateLpm = ((float)deltaPulses / FLOW_PULSES_PER_LITER) * (60000.0f / (float)dt);
    totalWaterLiters = (float)curTotal / FLOW_PULSES_PER_LITER;
    sessionWaterLiters = (float)curSession / FLOW_PULSES_PER_LITER;

    // ---------------------------------------------------------------
    // AUTO-DETEKSI SESI ALIRAN BARU
    // Jika air berhenti >= 5 detik lalu mengalir lagi di waktu berbeda,
    // volume sesi (sessionWaterLiters) otomatis di-reset ke 0
    // agar tampilan LCD & MQTT menunjukkan volume aliran terbaru saja.
    // Logika ini HANYA aktif saat FSM IDLE (tidak ada siklus irigasi/fertigasi).
    // ---------------------------------------------------------------
    if (actuatorState == STATE_IDLE) {
      if (flowRateLpm < 0.05f) {
        // Aliran berhenti atau sangat kecil → tandai sebagai idle
        if (!flowIsIdle) {
          flowIdleSince = now;
          flowIsIdle = true;
        }
      } else {
        // Aliran terdeteksi aktif
        if (flowIsIdle && (now - flowIdleSince >= FLOW_IDLE_THRESHOLD)) {
          // Transisi: Idle → Mengalir setelah jeda >= 5 detik
          // Ini adalah aliran baru di waktu berbeda → reset sesi otomatis
          noInterrupts();
          sessionFlowPulses = 0;
          interrupts();
          sessionWaterLiters = 0.0f;
          Serial.println(F("[FLOW] Aliran baru terdeteksi setelah idle. Sesi volume di-reset otomatis."));
        }
        flowIsIdle = false;
      }
    } else {
      // FSM sedang aktif (siklus irigasi/fertigasi berjalan), biarkan FSM mengelola sesi
      flowIsIdle = false;
      flowIdleSince = now;
    }
  }
}

void stopAllActuators(const char* reason = "SELESAI") {
  bool wasActive = statusIrigasi || statusFertigasi || statusPompa || (actuatorState != STATE_IDLE);

  digitalWrite(PIN_RELAY_IRIGASI, RELAY_INACTIVE_LEVEL);
  digitalWrite(PIN_RELAY_FERTIGASI, RELAY_INACTIVE_LEVEL);
  statusIrigasi = false;
  statusFertigasi = false;
  statusPompa = false;
  actuatorState = STATE_IDLE;
  queueFertigasiPending = false;
  is3PhaseFertCycle = false;
  isKarhutlaCycle = false;
  targetDurasiIrigasiDetik = 0;

  // Balas perintah OFF walau aktuator memang sudah mati. Web memakai status ini
  // sebagai konfirmasi bahwa ESP32 Gateway benar-benar menerima perintahnya.
  const bool isRemoteCommand = strncmp(reason, "CMD_", 4) == 0;
  if (wasActive || strcmp(reason, "INIT") == 0 || isRemoteCommand) {
    Serial.printf("[AKTUATOR] Seluruh katup ditutup (%s). Sesi: %.2f L | Total: %.2f L\n",
                  reason, sessionWaterLiters, totalWaterLiters);
    publishActuatorMqtt();
  }
}

void startIrigasiTerjadwal(int durasiDetik) {
  durasiDetik = constrain(durasiDetik, 2, 600); // Batas aman 2 detik s/d 10 menit
  targetDurasiIrigasiDetik = durasiDetik;
  targetLiterIrigasi = 999.0f; // Target liter tidak membatasi, batasan utamanya adalah durasi waktu
  queueFertigasiPending = false;
  isKarhutlaCycle = false;
  is3PhaseFertCycle = false;

  noInterrupts();
  sessionFlowPulses = 0;
  interrupts();
  sessionWaterLiters = 0.0f;

  sessionStartTime = millis();
  digitalWrite(PIN_RELAY_FERTIGASI, RELAY_INACTIVE_LEVEL);
  statusFertigasi = false;
  digitalWrite(PIN_RELAY_IRIGASI, RELAY_ACTIVE_LEVEL);
  statusIrigasi = true;
  statusPompa = true;
  actuatorState = STATE_IRIGASI_ACTIVE;

  publishActuatorMqtt();
  publishScheduleMqtt();
  Serial.printf("[JADWAL] >>> MEMULAI PENYIRAMAN TERJADWAL SELAMA %d DETIK <<<\n", durasiDetik);
  safeMqttPublish("sproutai/actuator/event", "{\"event\":\"SCHEDULED_WATERING_STARTED\"}");
}

void startIrigasi(float targetL, bool autoQueueFert = false, float nextFertTarget = 0.0f, bool karhutla = false) {
  digitalWrite(PIN_RELAY_FERTIGASI, RELAY_INACTIVE_LEVEL);
  statusFertigasi = false;

  targetLiterIrigasi = max(targetL, 0.2f);
  queueFertigasiPending = autoQueueFert;
  queuedFertigasiTarget = nextFertTarget;
  isKarhutlaCycle = karhutla;

  noInterrupts();
  sessionFlowPulses = 0;
  interrupts();
  sessionWaterLiters = 0.0f;

  sessionStartTime = millis();
  digitalWrite(PIN_RELAY_IRIGASI, RELAY_ACTIVE_LEVEL);
  statusIrigasi = true;
  statusPompa = true;
  actuatorState = STATE_IRIGASI_ACTIVE;

  publishActuatorMqtt();
}

void startFertigasi(float targetL, bool karhutla = false) {
  digitalWrite(PIN_RELAY_IRIGASI, RELAY_INACTIVE_LEVEL);
  statusIrigasi = false;

  targetLiterFertigasi = max(targetL, 0.2f);
  isKarhutlaCycle = karhutla;

  noInterrupts();
  sessionFlowPulses = 0;
  interrupts();
  sessionWaterLiters = 0.0f;

  sessionStartTime = millis();
  digitalWrite(PIN_RELAY_FERTIGASI, RELAY_ACTIVE_LEVEL);
  statusFertigasi = true;
  statusPompa = true;
  actuatorState = STATE_FERTIGASI_ACTIVE;

  publishActuatorMqtt();
}

void startFertigasiSiklus3Tahap(float volNutrisi, float customPreWet, float customPostFlush, bool karhutla) {
  // Volume Pre-Wet & Post-Flush:
  // Lahan biasa: Pre-wet 25-35% (min 0.3 L), Post-flush 20-25% (min 0.25 L)
  // Lahan gambut: Pre-wet masif/panjang (min 1.0 L untuk pecah hidrofobisitas), Post-flush 0.6 L
  float preWetL    = (customPreWet > 0.0f) ? customPreWet : (karhutla ? 1.0f : max(volNutrisi * 0.35f, 0.30f));
  float postFlushL = (customPostFlush > 0.0f) ? customPostFlush : (karhutla ? 0.6f : max(volNutrisi * 0.25f, 0.25f));

  targetLiterIrigasi    = preWetL;
  queuedFertigasiTarget = volNutrisi;
  targetLiterPostFlush  = postFlushL;
  queueFertigasiPending = true;
  is3PhaseFertCycle     = true;
  isKarhutlaCycle       = karhutla;

  // Nyalakan Tahap 1: Pre-Wet Air Bersih (Katup Irigasi ON)
  digitalWrite(PIN_RELAY_FERTIGASI, RELAY_INACTIVE_LEVEL);
  statusFertigasi = false;

  noInterrupts();
  sessionFlowPulses = 0;
  interrupts();
  sessionWaterLiters = 0.0f;

  sessionStartTime = millis();
  digitalWrite(PIN_RELAY_IRIGASI, RELAY_ACTIVE_LEVEL);
  statusIrigasi = true;
  statusPompa = true;
  actuatorState = STATE_IRIGASI_ACTIVE;

  publishActuatorMqtt();
  Serial.printf("[FERT-3PHASE] Mulai Siklus 3-Tahap (%s): Pre-Wet %.2f L -> Dosing %.2f L -> Flush %.2f L\n",
                karhutla ? "GAMBUT" : "HORTIKULTURA", preWetL, volNutrisi, postFlushL);
}

void processActuatorFSM() {
  unsigned long now = millis();
  const bool isManual = modeSistem.equalsIgnoreCase("MANUAL");

  switch (actuatorState) {
    case STATE_IDLE:
      break;

    case STATE_IRIGASI_ACTIVE: {
      // 1. Jika ini penyiraman terjadwal berbasis durasi detik:
      if (targetDurasiIrigasiDetik > 0) {
        if (now - sessionStartTime >= (unsigned long)targetDurasiIrigasiDetik * 1000UL) {
          targetDurasiIrigasiDetik = 0;
          stopAllActuators("JADWAL_DURASI_TERCAPAI");
          lastAutoCycleTime = now;
          Serial.println(F("[JADWAL] Penyiraman terjadwal selesai sesuai durasi!"));
          safeMqttPublish("sproutai/actuator/event", "{\"event\":\"SCHEDULED_WATERING_COMPLETED\"}");
          publishScheduleMqtt();
          return;
        }
      } else {
        // 2. Pada mode OTOMATIS (berbasis volume closed-loop):
        if (!isManual && (sessionWaterLiters >= targetLiterIrigasi)) {
          digitalWrite(PIN_RELAY_IRIGASI, RELAY_INACTIVE_LEVEL);
          statusIrigasi = false;

          if (queueFertigasiPending) {
            actuatorState = STATE_INTER_DELAY;
            interDelayStartTime = now;
            Serial.println(F("[FSM] Pre-Wet selesai. Jeda 3 detik sebelum Dosing fertigasi..."));
          } else {
            stopAllActuators("TARGET_IRIGASI_TERCAPAI");
            lastAutoCycleTime = now;
          }
          return;
        }
      }

      // 3. Batas Safety Timeout: jika terjadwal maka durasi + 10 detik, jika manual 30 menit, jika otomatis 3 menit
      unsigned long maxTimeout = (targetDurasiIrigasiDetik > 0) ?
                                 ((unsigned long)targetDurasiIrigasiDetik * 1000UL + 10000UL) :
                                 (isManual ? 1800000UL : 180000UL);
      if (now - sessionStartTime > maxTimeout) {
        targetDurasiIrigasiDetik = 0;
        stopAllActuators("SAFETY_TIMEOUT");
        return;
      }

      // 4. NO_FLOW_DETECTED: HANYA berlaku jika BUKAN irigasi durasi terjadwal dan mode otomatis
      if (!isManual && targetDurasiIrigasiDetik == 0 && (now - sessionStartTime > 35000UL) && (sessionWaterLiters < 0.03f)) {
        stopAllActuators("NO_FLOW_DETECTED");
        safeMqttPublish("sproutai/actuator/alarm", "{\"alarm\":\"NO_WATER_FLOW\",\"valve\":\"IRIGASI\"}");
        return;
      }
      break;
    }

    case STATE_INTER_DELAY: {
      if (now - interDelayStartTime >= 3000UL) {
        queueFertigasiPending = false;
        startFertigasi(queuedFertigasiTarget, isKarhutlaCycle);
      }
      break;
    }

    case STATE_FERTIGASI_ACTIVE: {
      // 1. Pada mode OTOMATIS, matikan jika target liter nutrisi tercapai
      if (!isManual && (sessionWaterLiters >= targetLiterFertigasi)) {
        digitalWrite(PIN_RELAY_FERTIGASI, RELAY_INACTIVE_LEVEL);
        statusFertigasi = false;

        if (is3PhaseFertCycle) {
          actuatorState = STATE_POST_FLUSH_DELAY;
          postFlushDelayStartTime = now;
          Serial.println(F("[FSM] Dosing pupuk selesai. Jeda 3 detik sebelum Post-Flush bilas pipa..."));
        } else {
          stopAllActuators("TARGET_FERTIGASI_TERCAPAI");
          lastAutoCycleTime = now;
        }
        return;
      }

      // 2. Batas Safety Timeout: 30 menit (Manual) atau 2 menit (Otomatis)
      unsigned long maxTimeout = isManual ? 1800000UL : 120000UL;
      if (now - sessionStartTime > maxTimeout) {
        stopAllActuators("SAFETY_TIMEOUT");
        return;
      }

      // 3. NO_FLOW_DETECTED: tandon nutrisi habis atau pompa macet
      if (!isManual && (now - sessionStartTime > 35000UL) && (sessionWaterLiters < 0.03f)) {
        stopAllActuators("NO_FLOW_DETECTED");
        safeMqttPublish("sproutai/actuator/alarm", "{\"alarm\":\"NO_FERTILIZER_FLOW\",\"valve\":\"FERTIGASI\"}");
        return;
      }
      break;
    }

    case STATE_POST_FLUSH_DELAY: {
      if (now - postFlushDelayStartTime >= 3000UL) {
        // Mulai Tahap 3: Post-Flush Bilas Pipa dengan Air Bersih (Katup Irigasi ON)
        noInterrupts();
        sessionFlowPulses = 0;
        interrupts();
        sessionWaterLiters = 0.0f;

        sessionStartTime = millis();
        digitalWrite(PIN_RELAY_IRIGASI, RELAY_ACTIVE_LEVEL);
        statusIrigasi = true;
        statusPompa = true;
        actuatorState = STATE_POST_FLUSH_ACTIVE;

        publishActuatorMqtt();
        Serial.printf("[FSM] Memulai Post-Flush bilas pipa %.2f L\n", targetLiterPostFlush);
      }
      break;
    }

    case STATE_POST_FLUSH_ACTIVE: {
      if (!isManual && (sessionWaterLiters >= targetLiterPostFlush)) {
        stopAllActuators("SIKLUS_3TAHAP_SELESAI");
        lastAutoCycleTime = now;
        lastFertigasiTimestamp = now;

        struct tm ti;
        if (getLocalTime(&ti, 20)) {
          lastFertigasiDay = ti.tm_mday;
          lastFertigasiMonth = ti.tm_mon + 1;
          lastFertigasiYear = ti.tm_year + 1900;
          saveAgriSettingsNVS();
        }

        Serial.println(F("[FSM] >>> SIKLUS 3-TAHAP FERTIGASI SUKSES LENGKAP & PIPA DIBILAS BERSIH! <<<"));
        safeMqttPublish("sproutai/actuator/event", "{\"event\":\"FERTIGATION_CYCLE_COMPLETED\"}");
        return;
      }

      unsigned long maxTimeout = isManual ? 1800000UL : 120000UL;
      if (now - sessionStartTime > maxTimeout) {
        stopAllActuators("SAFETY_TIMEOUT");
        return;
      }

      if (!isManual && (now - sessionStartTime > 35000UL) && (sessionWaterLiters < 0.03f)) {
        stopAllActuators("NO_FLOW_DETECTED");
        safeMqttPublish("sproutai/actuator/alarm", "{\"alarm\":\"NO_WATER_FLOW\",\"valve\":\"IRIGASI_FLUSH\"}");
        return;
      }
      break;
    }
  }
}

// =========================================================================
// SISTEM WAKTU NTP (LOCAL TIME WIB UTC+7) & GUARD FERTIGASI BIOLOGIS
// =========================================================================
bool getLocalTimeSafe(int &outHour, int &outMinute, int &outSecond) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 50)) {
    return false;
  }
  if (timeinfo.tm_year < (2020 - 1900)) {
    return false;
  }
  outHour = timeinfo.tm_hour;
  outMinute = timeinfo.tm_min;
  outSecond = timeinfo.tm_sec;
  return true;
}

bool getLocalTimeFull(int &outYear, int &outMonth, int &outDay, int &outHour, int &outMinute, int &outSecond) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 50)) {
    return false;
  }
  if (timeinfo.tm_year < (2020 - 1900)) {
    return false;
  }
  outYear   = timeinfo.tm_year + 1900;
  outMonth  = timeinfo.tm_mon + 1;
  outDay    = timeinfo.tm_mday;
  outHour   = timeinfo.tm_hour;
  outMinute = timeinfo.tm_min;
  outSecond = timeinfo.tm_sec;
  return true;
}

void publishScheduleMqtt() {
  if (!mqttClient.connected()) return;
  int curYr = 0, curMon = 0, curDy = 0, curH = -1, curM = -1, curS = -1;
  bool timeValid = getLocalTimeFull(curYr, curMon, curDy, curH, curM, curS);

  char curTimeStr[16];
  if (timeValid) {
    snprintf(curTimeStr, sizeof(curTimeStr), "%02d:%02d:%02d", curH, curM, curS);
  } else {
    snprintf(curTimeStr, sizeof(curTimeStr), "--:--:--");
  }

  char lastWateredStr[16];
  if (lastWateredHour >= 0 && lastWateredMinute >= 0) {
    snprintf(lastWateredStr, sizeof(lastWateredStr), "%02d:%02d", lastWateredHour, lastWateredMinute);
  } else {
    snprintf(lastWateredStr, sizeof(lastWateredStr), "Belum ada");
  }

  char schedJson[320];
  snprintf(schedJson, sizeof(schedJson),
    "{\"schedule_enabled\":%s,\"watering_time\":\"%02d:%02d\",\"watering_duration\":%d,"
    "\"time_valid\":%s,\"current_time\":\"%s\",\"last_watered_day\":%d,\"last_watered\":\"%s\"}",
    jadwalPenyiramanAktif ? "true" : "false",
    jadwalJam, jadwalMenit,
    jadwalPenyiramanDurasiDetik,
    timeValid ? "true" : "false",
    curTimeStr,
    lastWateredDay,
    lastWateredStr
  );
  safeMqttPublish("sproutai/schedule/status", schedJson, true);
}

// =========================================================================
// EVALUASI JADWAL PENYIRAMAN: EKSEKUSI INDEPENDEN TEPAT WAKTU & DURASI
// Berjalan otomatis setiap hari pada jam & menit yang ditentukan
// =========================================================================
void evaluasiJadwalPenyiraman() {
  if (!jadwalPenyiramanAktif) return;
  if (actuatorState != STATE_IDLE) return; // Tunggu jika aktuator sedang sibuk

  int curYr = 0, curMon = 0, curDy = 0, curH = -1, curM = -1, curS = -1;
  bool timeValid = getLocalTimeFull(curYr, curMon, curDy, curH, curM, curS);
  if (!timeValid) return; // Waktu RTC belum sinkron dari NTP/Web

  // Cek apakah jam dan menit saat ini tepat sesuai jadwal yang disetel
  if (curH == jadwalJam && curM == jadwalMenit) {
    // Pastikan hanya berjalan SATU KALI per hari pada jam & menit tersebut
    if (lastWateredDay != curDy || lastWateredHour != curH || lastWateredMinute != curM) {
      // PENGAMAN KESELAMATAN EKSTREM:
      // 1. Jika tanah sudah jenuh/kebanjiran (>= ambangAtas)
      if (!isnan(agriSoilRep) && agriSoilRep >= ambangAtas) {
        Serial.printf("[JADWAL] Pukul %02d:%02d WIB dilewati karena tanah sudah jenuh air (%.1f%% >= %.1f%%)\n",
                      curH, curM, agriSoilRep, ambangAtas);
        lastWateredDay = curDy;
        lastWateredHour = curH;
        lastWateredMinute = curM;
        publishScheduleMqtt();
        return;
      }
      // 2. Jika prakiraan cuaca hujan badai lebat BMKG (peluang >= 85%)
      if (peluangHujan >= 85.0f) {
        Serial.printf("[JADWAL] Pukul %02d:%02d WIB dilewati karena potensi hujan lebat (%.0f%%)\n",
                      curH, curM, peluangHujan);
        lastWateredDay = curDy;
        lastWateredHour = curH;
        lastWateredMinute = curM;
        publishScheduleMqtt();
        return;
      }

      // Catat bahwa jadwal hari ini telah dieksekusi
      lastWateredDay = curDy;
      lastWateredHour = curH;
      lastWateredMinute = curM;

      int durasi = (jadwalPenyiramanDurasiDetik > 0) ? jadwalPenyiramanDurasiDetik : 10;
      Serial.printf("[JADWAL TIBA] Pukul %02d:%02d WIB! Memulai penyiraman selama %d detik...\n",
                    curH, curM, durasi);
      startIrigasiTerjadwal(durasi);
    }
  }
}

// Jendela Biologis Transpirasi Tanaman:
// Pagi: 06:30 - 08:30 WIB | Sore: 16:00 - 17:30 WIB
bool isJendelaBiologis(int hour, int minute) {
  bool pagi = (hour == 6 && minute >= 30) || (hour == 7) || (hour == 8 && minute <= 30);
  bool sore = (hour == 16) || (hour == 17 && minute <= 30);
  return (pagi || sore);
}

// Blokir Siang Hari Terik: 11:00 - 14:00 WIB (Cegah Evaporasi Ekstrem & Root Burn)
bool isSiangTerik(int hour) {
  return (hour >= 11 && hour < 14);
}

// Proteksi Keamanan Sensor pH Tandon (Quality Control Larutan Racikan User)
bool isPhLarutanBerbahaya(float ph) {
  if (isnan(ph)) return false;
  // Jika pH di bawah 4.0 (asam ekstrem) atau di atas 8.5 (basa pekat beracun bagi tanaman)
  return (ph < 4.0f || ph > 8.5f);
}

// =========================================================================
// PENGATURAN AMBANG BERDASARKAN FASE TANAMAN (VEGETATIF VS GENERATIF)
// =========================================================================
void updateAmbangFase() {
  if (faseVegetatif) {
    ambangSkorAktif = 55.0f; // Vegetatif: kebutuhan air lebih stabil & sering
    ambangSkorBawah = 25.0f; // Histeresis batas bawah pematian
  } else {
    ambangSkorAktif = 60.0f; // Generatif: lebih selektif untuk hindari pembusukan bunga/buah
    ambangSkorBawah = 35.0f;
  }
}

// =========================================================================
// SIMPAN & MUAT PENGATURAN HORTIKULTURA DARI NVS FLASH
// =========================================================================
void loadAgriSettingsNVS() {
  if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (prefs.begin("tn_agri", true)) { // Read-only
      faseVegetatif = prefs.getBool("fase_veg", true);
      ambangAtas = prefs.getFloat("th_atas", 75.0f);
      ambangBawah = prefs.getFloat("th_bawah", 45.0f);
      ambangKritis = prefs.getFloat("th_kritis", 35.0f);
      ambangTempLow = prefs.getFloat("th_t_low", 22.0f);
      ambangTempHigh = prefs.getFloat("th_t_hi", 34.0f);
      ambangHumLow = prefs.getFloat("th_h_low", 60.0f);
      ambangHumHigh = prefs.getFloat("th_h_hi", 85.0f);
      jadwalPenyiramanAktif = prefs.getBool("sched_en", true);
      jadwalJam = prefs.getInt("sched_jam", 6);
      jadwalMenit = prefs.getInt("sched_menit", 0);
      jadwalPenyiramanDurasiDetik = prefs.getInt("sched_dur", 10);
      peluangHujan = prefs.getFloat("rain_p", 0.0f);
      intervalHariFertigasi = prefs.getInt("fert_int", 2);
      lastFertigasiDay = prefs.getInt("last_f_day", -1);
      lastFertigasiMonth = prefs.getInt("last_f_mon", -1);
      lastFertigasiYear = prefs.getInt("last_f_yr", -1);
      prefs.end();
    }
    xSemaphoreGive(xPrefsMutex);
  }
  updateAmbangFase();
  Serial.printf("[AGRI CFG] Fase: %s | Tanah: Atas=%.1f%% Bawah=%.1f%% Kritis=%.1f%% | Suhu: %.1f-%.1f C | RH: %.1f-%.1f%% | Jadwal: %02d:%02d (%s) | Fert Interval: %d hari (Terakhir: %02d/%02d)\n",
                faseVegetatif ? "VEGETATIF" : "GENERATIF",
                ambangAtas, ambangBawah, ambangKritis,
                ambangTempLow, ambangTempHigh, ambangHumLow, ambangHumHigh,
                jadwalJam, jadwalMenit, jadwalPenyiramanAktif ? "AKTIF" : "NONAKTIF",
                intervalHariFertigasi, lastFertigasiDay, lastFertigasiMonth);
}

void saveAgriSettingsNVS() {
  if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (prefs.begin("tn_agri", false)) { // Read-Write
      prefs.putBool("fase_veg", faseVegetatif);
      prefs.putFloat("th_atas", ambangAtas);
      prefs.putFloat("th_bawah", ambangBawah);
      prefs.putFloat("th_kritis", ambangKritis);
      prefs.putFloat("th_t_low", ambangTempLow);
      prefs.putFloat("th_t_hi", ambangTempHigh);
      prefs.putFloat("th_h_low", ambangHumLow);
      prefs.putFloat("th_h_hi", ambangHumHigh);
      prefs.putBool("sched_en", jadwalPenyiramanAktif);
      prefs.putInt("sched_jam", jadwalJam);
      prefs.putInt("sched_menit", jadwalMenit);
      prefs.putInt("sched_dur", jadwalPenyiramanDurasiDetik);
      prefs.putFloat("rain_p", peluangHujan);
      prefs.putInt("fert_int", intervalHariFertigasi);
      prefs.putInt("last_f_day", lastFertigasiDay);
      prefs.putInt("last_f_mon", lastFertigasiMonth);
      prefs.putInt("last_f_yr", lastFertigasiYear);
      prefs.end();
    }
    xSemaphoreGive(xPrefsMutex);
  }
}

// =========================================================================
// PENGGABUNGAN DATA 2 NODE HORTIKULTURA (NODE 1 & NODE 2)
// Menangani Deep Sleep, Keandalan Sensor, dan Disparitas Kelembapan Lahan
// =========================================================================
void hitungMetrikDuaNode(float &outSoilRep, float &outSoilMin, float &outTemp, float &outRh) {
  unsigned long now = millis();
  // Toleransi 180 detik mencakup siklus Deep Sleep Node 1 & Node 2 (5-30s + transmisi)
  bool n1Valid = (node1.active && (node1.lastSeen > 0) && (now - node1.lastSeen < 180000UL) && !isnan(node1.soil_moisture));
  bool n2Valid = (node2.active && (node2.lastSeen > 0) && (now - node2.lastSeen < 180000UL) && !isnan(node2.soil_moisture));

  float s1 = n1Valid ? node1.soil_moisture : (!isnan(node1.soil_moisture) ? node1.soil_moisture : NAN);
  float s2 = n2Valid ? node2.soil_moisture : (!isnan(node2.soil_moisture) ? node2.soil_moisture : NAN);

  // 1. Kelembapan Tanah Terendah & Representatif
  if (!isnan(s1) && !isnan(s2)) {
    outSoilMin = min(s1, s2);
    // Jika ada ketimpangan kelembapan antar dua titik (> 8%), bobotkan ke titik terkering (70% min + 30% avg)
    float selisih = fabs(s1 - s2);
    if (selisih > 8.0f) {
      outSoilRep = (outSoilMin * 0.7f) + (((s1 + s2) / 2.0f) * 0.3f);
    } else {
      outSoilRep = (s1 + s2) / 2.0f;
    }
  } else if (!isnan(s1)) {
    outSoilMin = s1;
    outSoilRep = s1;
  } else if (!isnan(s2)) {
    outSoilMin = s2;
    outSoilRep = s2;
  } else {
    outSoilMin = 50.0f;
    outSoilRep = 50.0f;
  }

  // 2. Mikroklimat Gabungan Suhu & Kelembapan Udara untuk VPD
  float t1 = (!isnan(node1.temperature) && node1.temperature > 0.0f) ? node1.temperature : NAN;
  float t2 = (!isnan(node2.temperature) && node2.temperature > 0.0f) ? node2.temperature : NAN;
  if (!isnan(t1) && !isnan(t2)) outTemp = (t1 + t2) / 2.0f;
  else if (!isnan(t1)) outTemp = t1;
  else if (!isnan(t2)) outTemp = t2;
  else outTemp = 28.0f;

  float rh1 = (!isnan(node1.humidity) && node1.humidity > 0.0f) ? node1.humidity : NAN;
  float rh2 = (!isnan(node2.humidity) && node2.humidity > 0.0f) ? node2.humidity : NAN;
  if (!isnan(rh1) && !isnan(rh2)) outRh = (rh1 + rh2) / 2.0f;
  else if (!isnan(rh1)) outRh = rh1;
  else if (!isnan(rh2)) outRh = rh2;
  else outRh = 65.0f;
}

// =========================================================================
// RUMUS PERHITUNGAN SKOR IRIGASI LENGKAP (SESUAI FORMULA ARDUINO CODE NEXA)
// =========================================================================
void hitungSkorIrigasiHortikultura(float tanah, float suhu, float rh, float p_hujan) {
  if (isnan(tanah) || isnan(suhu) || isnan(rh) || suhu <= 0.0f || rh <= 0.0f) return;

  // 1. Perhitungan VPD (Tetens Equation, kPa)
  float svp = 0.6108f * expf((17.27f * suhu) / (suhu + 237.3f));
  vpdAgri = svp * (1.0f - (rh / 100.0f));
  vpdAgri = constrain(vpdAgri, 0.4f, 2.0f);

  // 2. Skor VPD: 0 - 30 poin (Makin tinggi VPD = udara kering & panas = tuntutan air naik)
  skorVpd = ((vpdAgri - 0.4f) * 30.0f) / (2.0f - 0.4f);
  skorVpd = constrain(skorVpd, 0.0f, 30.0f);

  // 3. Skor Tanah: 0 - 50 poin (Linear defisit dari ambangAtas ke ambangKritis)
  float batasAtas = ambangAtas;
  float batasKritis = ambangKritis;
  if (batasAtas <= batasKritis) batasAtas = batasKritis + 1.0f;

  skorTanah = ((batasAtas - tanah) * 50.0f) / (batasAtas - batasKritis);
  skorTanah = constrain(skorTanah, 0.0f, 50.0f);

  // 4. Skor Peluang Hujan BMKG: 0 - 40 poin (Pengurang skor untuk hemat air)
  float safeRain = constrain(p_hujan, 0.0f, 100.0f);
  skorHujan = (safeRain / 100.0f) * 40.0f;
  skorHujan = constrain(skorHujan, 0.0f, 40.0f);

  // 5. Skor Interaksi (Maks 20 Poin agar Total Skor Maksimal Pas Tepat 100 Poin)
  // Ketika skorTanah=50 dan skorVpd=30: (50 * 30) / 75.0 = 20.0 poin
  skorInteraksi = ((skorTanah * skorVpd) / 75.0f);
  skorInteraksi = constrain(skorInteraksi, 0.0f, 20.0f);

  // 6. Skor Total Akhir (Rentang Standar Presisi 0.0 s/d 100.0 Poin: 50 + 30 + 20 - 0 = 100)
  skorTotal = constrain(skorTanah + skorVpd + skorInteraksi - skorHujan, 0.0f, 100.0f);

  // 7. Update Status Siklus Irigasi (Histeresis)
  if (skorTotal >= ambangSkorAktif) {
    siklusIrigasiAktif = true;
  } else if (skorTotal <= ambangSkorBawah) {
    siklusIrigasiAktif = false;
  }
}

// =========================================================================
// KALKULASI TARGET VOLUME AIR BERBASIS WATERFLOW (CLOSED-LOOP PRESISI)
// Efisiensi air maksimal: durasi penyiraman dikendalikan oleh volume air riil
// =========================================================================
float hitungTargetVolumeAir(float tanah, float vpd, float skor, float suhu = NAN, float rh = NAN) {
  float deltaSM = max(0.0f, ambangAtas - tanah);
  if (deltaSM <= 0.0f) return 0.0f;

  // Faktor evaporatif mikro-klimat dari VPD (0.8 - 1.6)
  float evapFactor = 1.0f + ((vpd - 0.8f) * 0.4f);

  // Kompensasi Stres Mikro-Klimat:
  // Jika suhu aktual melebihi ambang batas atas suhu atau RH di bawah ambang batas bawah,
  // laju transpirasi tanaman dan penguapan tanah sangat tinggi, naikkan kompensasi volume air
  if (!isnan(suhu) && suhu > ambangTempHigh) {
    evapFactor += 0.15f;
  }
  if (!isnan(rh) && rh < ambangHumLow) {
    evapFactor += 0.10f;
  }
  evapFactor = constrain(evapFactor, 0.8f, 1.8f);

  // Faktor fase tanaman (Vegetatif = 1.0x, Generatif = 0.85x)
  float faseFactor = faseVegetatif ? 1.0f : 0.85f;

  // Skala kebutuhan air berbasis skor (skor 30 -> 0.6x, skor 60 -> 1.2x)
  float skorFactor = constrain(skor / 50.0f, 0.6f, 1.5f);

  // Target volume air per siklus: 0.4 s/d 3.5 Liter
  float volumeTarget = (deltaSM / 30.0f) * 1.5f * evapFactor * faseFactor * skorFactor;
  return constrain(volumeTarget, 0.4f, 3.5f);
}

float hitungTargetFertigasiPertanian(float ph) {
  if (isnan(ph)) return 0.8f;
  float deviasiPh = fabs(ph - 6.5f);
  float volumeTarget = 0.8f * (1.0f + (deviasiPh * 0.35f));
  return constrain(volumeTarget, 0.3f, 2.5f);
}

// =========================================================================
// PUBLIKASI TELEMETRI SKOR IRIGASI KE MQTT (WEB DASHBOARD VISIBILITY)
// =========================================================================
void publishIrrigationScoreMqtt() {
  if (!mqttClient.connected()) return;

  char scoreJson[440];
  snprintf(scoreJson, sizeof(scoreJson),
    "{"
    "\"skor_tanah\":%.1f,"
    "\"skor_vpd\":%.1f,"
    "\"skor_interaksi\":%.1f,"
    "\"skor_hujan\":%.1f,"
    "\"skor_total\":%.1f,"
    "\"vpd\":%.2f,"
    "\"fase\":\"%s\","
    "\"ambang_aktif\":%.1f,"
    "\"ambang_bawah\":%.1f,"
    "\"siklus_aktif\":%s,"
    "\"soil_rep\":%.1f,"
    "\"soil_min\":%.1f,"
    "\"target_liters\":%.2f,"
    "\"peluang_hujan\":%.1f,"
    "\"jadwal\":\"%02d:%02d\","
    "\"jadwal_aktif\":%s,"
    "\"fert_interval\":%d,"
    "\"last_fert_day\":%d"
    "}",
    skorTanah, skorVpd, skorInteraksi, skorHujan, skorTotal,
    vpdAgri,
    faseVegetatif ? "vegetatif" : "generatif",
    ambangSkorAktif, ambangSkorBawah,
    siklusIrigasiAktif ? "true" : "false",
    isnan(agriSoilRep) ? 0.0f : agriSoilRep,
    isnan(agriSoilMin) ? 0.0f : agriSoilMin,
    (actuatorState == STATE_IRIGASI_ACTIVE) ? targetLiterIrigasi :
    (actuatorState == STATE_FERTIGASI_ACTIVE) ? targetLiterFertigasi :
    (actuatorState == STATE_POST_FLUSH_ACTIVE) ? targetLiterPostFlush : 0.0f,
    peluangHujan,
    jadwalJam, jadwalMenit,
    jadwalPenyiramanAktif ? "true" : "false",
    intervalHariFertigasi,
    lastFertigasiDay
  );

  safeMqttPublish("sproutai/irrigation/score", scoreJson, false);
}

// =========================================================================
// EVALUASI AKTUATOR OTOMATIS: INTEGRASI LOGIKA SKOR, JADWAL, WATERFLOW
// DAN KONTROL FERTIGASI CERDAS (HYBRID HORTIKULTURA & LAHAN GAMBUT)
// =========================================================================
void evaluasiAktuatorOtomatis() {
  if (!modeSistem.equalsIgnoreCase("OTOMATIS")) return;
  if (actuatorState != STATE_IDLE) return;

  unsigned long now = millis();

  // Waktu lokal saat ini
  int curYr = 0, curMon = 0, curDy = 0, curH = -1, curM = -1, curS = -1;
  bool timeValid = getLocalTimeFull(curYr, curMon, curDy, curH, curM, curS);

  // -----------------------------------------------------------------------
  // PRIORITAS 1: MITIGASI KARHUTLA GAMBUT (KONDISI B: NODE 3 AKTIF & ONLINE)
  // Menangani Lahan Gambut: Bahaya Bara Api Bawah Tanah vs Ameliorasi Masam
  // -----------------------------------------------------------------------
  bool node3Online = (node3.active && (node3.lastSeen > 0) && (now - node3.lastSeen < 60000UL));

  if (node3Online) {
    // 1A. SUB-KONDISI DARURAT BARA GAMBUT (Top Priority - Bypass Jam & Cuaca!)
    bool smolderingWarning = (!isnan(node3.soil_temp) && node3.soil_temp >= 40.0f);
    bool fwiBahaya         = (node3.fwi_level >= 2);

    if (smolderingWarning || fwiBahaya) {
      if (now - lastAutoCycleTime >= COOLDOWN_KARHUTLA_MS) {
        float targetRewetting = 3.0f;
        if (node3.fwi_level == 3) targetRewetting = 5.0f;
        if (node3.fwi_level >= 4 || (!isnan(node3.soil_temp) && node3.soil_temp >= 50.0f)) {
          targetRewetting = 8.0f;
        }

        // Fertigasi penetral keasaman arang gambut (Amelioran Kapur Dolomit Cair)
        float targetAmelioran = 1.5f;
        if (!isnan(gatewayPh) && gatewayPh < 4.8f) {
          targetAmelioran = 2.5f;
        }
        targetAmelioran = min(targetAmelioran, MAX_FERTIGASI_LITER_GAMBUT);

        Serial.printf("[AUTO GAMBUT DARURAT] Terdeteksi bahaya bara/FWI! Rewetting %.1f L + Amelioran %.1f L (Siklus 3-Tahap)...\n",
                      targetRewetting, targetAmelioran);
        // Siklus 3-Tahap Gambut: Pre-Wet Air Masif -> Injeksi Amelioran Masif -> Post-Flush Bilas
        startFertigasiSiklus3Tahap(targetAmelioran, targetRewetting, 0.8f, true);
        return;
      }
    }

    // 1B. SUB-KONDISI PROTEKSI KEJENUHAN GAMBUT (Wajib Mati jika Tergenang/Banjir)
    if (!isnan(node1.soil_moisture) && node1.soil_moisture >= 85.0f) {
      return; // Lahan gambut jenuh air/banjir, dilarang menyiram
    }
  }

  // -----------------------------------------------------------------------
  // PERSIAPAN DATA HORTIKULTURA: AGREGASI 2 NODE & HITUNG SKOR KEBUTUHAN AIR
  // -----------------------------------------------------------------------
  float soilRep = NAN;
  float soilMin = NAN;
  float tempAgri = NAN;
  float rhAgri = NAN;

  if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    hitungMetrikDuaNode(soilRep, soilMin, tempAgri, rhAgri);
    xSemaphoreGive(xDataMutex);
  }

  agriSoilRep = soilRep;
  agriSoilMin = soilMin;
  agriTemp    = tempAgri;
  agriRh      = rhAgri;

  // Jalankan rumus scoring irigasi (VPD, Tanah, Interaksi, Peluang Hujan)
  hitungSkorIrigasiHortikultura(agriSoilRep, agriTemp, agriRh, peluangHujan);

  // -----------------------------------------------------------------------
  // PRIORITAS 2: PROTEKSI DARURAT KEKERINGAN KRITIS (EMERGENCY OVERRIDE AIR BERSIH)
  // Jika salah satu titik tanah jatuh <= batas kritis, bypass jadwal jam!
  // Tanaman disiram air bersih murni untuk penyelamatan dehidrasi akut.
  // -----------------------------------------------------------------------
  if (!isnan(agriSoilMin) && (agriSoilMin <= ambangKritis) && (agriSoilMin >= 0.0f)) {
    if (now - lastAutoCycleTime >= 5UL * 60UL * 1000UL) { // Cooldown darurat 5 menit
      float volDarurat = hitungTargetVolumeAir(agriSoilMin, vpdAgri, 65.0f, agriTemp, agriRh);
      volDarurat = max(volDarurat, 1.2f); // Minimal 1.2 L untuk penyelamatan

      Serial.printf("[AUTO EMERGENCY] Tanah kritis (%.1f%% <= %.1f%%)! Siram darurat air murni %.2f L\n",
                    agriSoilMin, ambangKritis, volDarurat);
      startIrigasi(volDarurat, false, 0.0f, false);
      return;
    }
  }

  // -----------------------------------------------------------------------
  // PRIORITAS 3: PROTEKSI KEJENUHAN AIR & BADAI HUJAN DERAS
  // Jika tanah sudah jenuh (>= ambangAtas) atau peluang hujan >= 85%, batalkan!
  // -----------------------------------------------------------------------
  if ((!isnan(agriSoilRep) && (agriSoilRep >= ambangAtas)) || (peluangHujan >= 85.0f)) {
    return; // Air sudah mencukupi atau akan hujan deras
  }

  // -----------------------------------------------------------------------
  // PRIORITAS 4: KONTROL PENJADWALAN & KEPUTUSAN FERTIGASI NUTRISI VS IRIGASI AIR
  // -----------------------------------------------------------------------
  // 4A. EVALUASI GUARD "WAJIB MATI" FERTIGASI (INTERLOCK PENGAMAN)
  bool blokirFertigasi = false;
  const char* alasanBlokirFert = "AMAN";

  // Guard 1: Peluang hujan BMKG >= 60% (Cegah Pencucian Pupuk / Leaching ke Air Tanah)
  if (peluangHujan >= 60.0f) {
    blokirFertigasi = true;
    alasanBlokirFert = "PELUANG_HUJAN_TINGGI_CEGAH_LEACHING";
  }
  // Guard 2: Tanah sudah cukup basah (>= ambangAtas)
  else if (!isnan(agriSoilRep) && (agriSoilRep >= ambangAtas)) {
    blokirFertigasi = true;
    alasanBlokirFert = "TANAH_SUDAH_BASAH_CEGAH_BUSUK_AKAR";
  }
  // Guard 3: Blokir Siang Hari Terik (11:00 - 14:00 WIB)
  else if (timeValid && isSiangTerik(curH)) {
    blokirFertigasi = true;
    alasanBlokirFert = "SIANG_TERIK_11_14_CEGAH_ROOT_BURN";
  }
  // Guard 4: Kualitas Larutan Tandon Berbahaya (pH < 4.0 atau pH > 8.5)
  else if (isPhLarutanBerbahaya(gatewayPh)) {
    blokirFertigasi = true;
    alasanBlokirFert = "PH_TANDON_BERBAHAYA_OVERDOSIS";
    safeMqttPublish("sproutai/actuator/alarm", "{\"alarm\":\"PH_TANDON_DANGEROUS\",\"ph\":gatewayPh}");
  }

  // 4B. EVALUASI SYARAT "WAJIB NYALA" FERTIGASI (INTERVAL HARI & JENDELA BIOLOGIS)
  bool hariFertigasiTiba = false;
  if (timeValid) {
    if (lastFertigasiDay == -1) {
      hariFertigasiTiba = true; // Belum pernah fertigasi sejak boot/nvs awal
    } else if (lastFertigasiDay != curDy || lastFertigasiMonth != curMon) {
      int selisihHari = (curDy >= lastFertigasiDay) ? (curDy - lastFertigasiDay) : (curDy + 30 - lastFertigasiDay);
      if (selisihHari >= intervalHariFertigasi) {
        hariFertigasiTiba = true;
      }
    }
  } else {
    // Fallback bila waktu NTP belum sinkron
    if (lastFertigasiTimestamp == 0 || (now - lastFertigasiTimestamp >= (intervalHariFertigasi * 24UL * 3600UL * 1000UL))) {
      hariFertigasiTiba = true;
    }
  }

  bool dalamJendelaBiologis = timeValid ? isJendelaBiologis(curH, curM) : true;
  // 4C. EKSEKUSI CERDAS: FERTIGASI SIKLUS 3-TAHAP vs IRIGASI AIR BIASA (RESPON SENSORIK)
  // Beraksi jika cooldown tercapai DAN (siklus histeresis aktif ATAU skor total >= ambang batas)
  bool waktuAksiTiba = false;
  if ((now - lastAutoCycleTime >= COOLDOWN_AGRI_MS) && (siklusIrigasiAktif || (skorTotal >= ambangSkorBawah))) {
    waktuAksiTiba = true;
  }

  if (waktuAksiTiba) {
    // Kondisi khusus tanah gambut masam (Node 3 online & pH < 4.8):
    bool isGambutMasam = (node3Online && !isnan(gatewayPh) && gatewayPh < 4.8f);

    // Cek apakah memenuhi syarat fertigasi
    bool bolehFertigasi = (!blokirFertigasi && hariFertigasiTiba && dalamJendelaBiologis);
    if (isGambutMasam && !blokirFertigasi) {
      bolehFertigasi = true; // Pada gambut masam, prioritaskan penetralan asam
    }

    if (bolehFertigasi) {
      // =================================================================
      // EKSEKUSI SIKLUS 3-TAHAP FERTIGASI PRESISI
      // Pre-Wet (Air Bersih) -> Active Dosing (Pupuk/Amelioran) -> Post-Flush (Bilas Pipa)
      // =================================================================
      float volNutrisi = hitungTargetFertigasiPertanian(gatewayPh);

      // Modulasi Dosis Berdasarkan Fase Tanaman
      if (faseVegetatif) {
        volNutrisi = constrain(volNutrisi * 0.90f, 0.40f, 1.20f); // Vegetatif: nitrogen, dosis lebih encer
      } else {
        volNutrisi = constrain(volNutrisi * 1.15f, 0.60f, 1.80f); // Generatif: kalium/fosfat, dosis mantap
      }

      // Koreksi Tambahan untuk Lahan Gambut
      if (isGambutMasam) {
        volNutrisi = constrain(volNutrisi * 1.40f, 1.0f, MAX_FERTIGASI_LITER_GAMBUT);
      } else {
        volNutrisi = min(volNutrisi, MAX_FERTIGASI_LITER_NORMAL);
      }

      Serial.printf("[AUTO FERTIGASI 3-TAHAP] Pukul %02d:%02d | JendelaBiologis=%s | IntervalTiba=%s | Fase=%s | pH=%.2f | Target Dosing=%.2f L\n",
                    curH, curM, dalamJendelaBiologis ? "YA" : "TIDAK", hariFertigasiTiba ? "YA" : "TIDAK",
                    faseVegetatif ? "VEGETATIF" : "GENERATIF", gatewayPh, volNutrisi);

      startFertigasiSiklus3Tahap(volNutrisi, 0.0f, 0.0f, isGambutMasam);
      return;
    } else {
      // =================================================================
      // EKSEKUSI IRIGASI AIR BIASA
      // Pupuk di-skip/ditunda karena cuaca hujan, siang terik, atau bukan interval hari pupuk
      // Tanaman tetap disiram air bersih terukur agar tidak kekeringan!
      // =================================================================
      float volIrigasi = hitungTargetVolumeAir(agriSoilRep, vpdAgri, skorTotal, agriTemp, agriRh);
      if (volIrigasi >= 0.30f) {
        Serial.printf("[AUTO IRIGASI AIR BIASA] Fertigasi di-skip (%s). Menjalankan irigasi air bersih presisi %.2f L\n",
                      alasanBlokirFert, volIrigasi);
        startIrigasi(volIrigasi, false, 0.0f, false);
        return;
      }
    }
  }
}

// =========================================================================
// TFT DISPLAY RENDERING (FULL DASHBOARD: NODE 1, 2, 3, WATERFLOW, SENSOR pH)
// RESILIENT DISPLAY: TETAP AKTIF DAN MENAMPILKAN SEMUA DATA SAAT OFFLINE
// =========================================================================
unsigned long lastDisplayUpdate = 0;
static String lastRenderedMode = "";
static int lastRenderedNetState = -1;

void drawStaticDashboard() {
  tft.fillScreen(COLOR_BG);

  // 1. Header Bar (y: 0 - 36)
  tft.fillRect(0, 0, 480, 36, COLOR_HEADER);
  tft.drawFastHLine(0, 36, 480, 0x028A);
  tft.setTextColor(COLOR_TEXT, COLOR_HEADER);
  tft.setTextSize(2);
  tft.drawString("TERANOVA", 10, 10);
  tft.setTextColor(COLOR_CYAN, COLOR_HEADER);
  tft.drawString("GW", 112, 10);

  // 2. Card 1: Node 1 (Kiri Atas: x=6, y=40, w=232, h=124)
  tft.fillRoundRect(6, 40, 232, 124, 6, COLOR_CARD_BG);
  tft.drawRoundRect(6, 40, 232, 124, 6, COLOR_BORDER_N1);
  tft.setTextColor(COLOR_ACCENT, COLOR_CARD_BG);
  tft.setTextSize(1);
  tft.drawString("NODE 1 : HORTI A", 14, 46);
  tft.drawFastHLine(10, 58, 224, 0x18C3);
  tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
  tft.drawString("Suhu Udara   :", 14, 64);
  tft.drawString("Lembap Udara :", 14, 82);
  tft.drawString("Lembap Tanah :", 14, 100);
  tft.drawString("Baterai Node :", 14, 118);
  tft.drawString("Pkt Terakhir :", 14, 136);

  // 3. Card 2: Node 2 (Kanan Atas: x=242, y=40, w=232, h=124)
  tft.fillRoundRect(242, 40, 232, 124, 6, COLOR_CARD_BG);
  tft.drawRoundRect(242, 40, 232, 124, 6, COLOR_BORDER_N2);
  tft.setTextColor(COLOR_CYAN, COLOR_CARD_BG);
  tft.setTextSize(1);
  tft.drawString("NODE 2 : HORTI B", 250, 46);
  tft.drawFastHLine(246, 58, 224, 0x18C3);
  tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
  tft.drawString("Suhu Udara   :", 250, 64);
  tft.drawString("Lembap Udara :", 250, 82);
  tft.drawString("Lembap Tanah :", 250, 100);
  tft.drawString("Baterai Node :", 250, 118);
  tft.drawString("Pkt Terakhir :", 250, 136);

  // 4. Card 3: Node 3 Gambut & Karhutla (Kiri Bawah: x=6, y=168, w=232, h=126)
  tft.fillRoundRect(6, 168, 232, 126, 6, COLOR_CARD_BG);
  tft.drawRoundRect(6, 168, 232, 126, 6, COLOR_BORDER_N3);
  tft.setTextColor(COLOR_ORANGE, COLOR_CARD_BG);
  tft.setTextSize(1);
  tft.drawString("NODE 3 : GAMBUT", 14, 174);
  tft.drawFastHLine(10, 186, 224, 0x18C3);
  tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
  tft.drawString("Suhu Gambut  :", 14, 192);
  tft.drawString("Bara/Smolder :", 14, 210);
  tft.drawString("FWI BMKG     :", 14, 228);
  tft.drawString("Indeks FDRS  :", 14, 246);
  tft.drawString("Baterai/Pkt  :", 14, 264);
  // 5. Card 4: Water Flow, Sensor pH, & Pompa (Kanan Bawah: x=242, y=168, w=232, h=126)
  tft.fillRoundRect(242, 168, 232, 126, 6, COLOR_CARD_BG);
  tft.drawRoundRect(242, 168, 232, 126, 6, COLOR_BORDER_WTR);
  tft.setTextColor(COLOR_YELLOW, COLOR_CARD_BG);
  tft.setTextSize(1);
  tft.drawString("WATER FLOW & pH", 250, 174);
  tft.drawFastHLine(246, 186, 224, 0x18C3);
  tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
  tft.drawString("Sensor pH    :", 250, 192);
  tft.drawString("Debit Aliran :", 250, 210);
  tft.drawString("Volume Sesi  :", 250, 228);
  tft.drawString("Total Air    :", 250, 246);
  tft.drawString("Status Pompa :", 250, 264);

  // 6. Footer Bar (y: 298 - 320)
  tft.fillRect(0, 298, 480, 22, 0x0000);
  tft.drawFastHLine(0, 297, 480, 0x028A);

  // Paksa render ulang status badge pada refresh pertama
  lastRenderedMode = "";
  lastRenderedNetState = -1;
}

void updateDynamicValues() {
  // A. UPDATE STATUS MODE (MANUAL / OTOMATIS) DI HEADER
  if (modeSistem != lastRenderedMode) {
    lastRenderedMode = modeSistem;
    bool isAuto = modeSistem.equalsIgnoreCase("OTOMATIS");
    uint16_t boxBg     = isAuto ? 0x0283 : 0x01A5;
    uint16_t boxBorder = isAuto ? COLOR_ACCENT : COLOR_CYAN;
    uint16_t textColor = isAuto ? COLOR_ACCENT : COLOR_CYAN;

    tft.fillRoundRect(165, 5, 140, 26, 4, boxBg);
    tft.drawRoundRect(165, 5, 140, 26, 4, boxBorder);
    tft.setTextSize(2);
    tft.setTextColor(textColor, boxBg);
    if (isAuto) {
      tft.drawString("OTOMATIS", 183, 10);
    } else {
      tft.drawString(" MANUAL ", 183, 10);
    }
  }

  // B. UPDATE STATUS ESP32 (ONLINE / LOKAL / OFFLINE) DI HEADER
  int netState = 0;
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    netState = 2; // ONLINE PENUH (WiFi & HiveMQ Cloud Terhubung)
  } else if (WiFi.status() == WL_CONNECTED) {
    netState = 1; // LOKAL (WiFi Terhubung, Internet/Cloud Standby)
  } else {
    netState = 0; // OFFLINE (WiFi Terputus / Mandiri di Lapangan)
  }

  if (netState != lastRenderedNetState) {
    lastRenderedNetState = netState;
    uint16_t boxBg     = (netState == 2) ? 0x0283 : (netState == 1) ? 0x3180 : 0x3800;
    uint16_t boxBorder = (netState == 2) ? COLOR_ACCENT : (netState == 1) ? COLOR_YELLOW : COLOR_RED;
    uint16_t textColor = (netState == 2) ? COLOR_ACCENT : (netState == 1) ? COLOR_YELLOW : COLOR_RED;

    tft.fillRoundRect(315, 5, 155, 26, 4, boxBg);
    tft.drawRoundRect(315, 5, 155, 26, 4, boxBorder);
    tft.setTextSize(2);
    tft.setTextColor(textColor, boxBg);
    if (netState == 2) {
      tft.drawString("  ONLINE  ", 335, 10);
    } else if (netState == 1) {
      tft.drawString("  LOKAL   ", 335, 10);
    } else {
      tft.drawString(" OFFLINE  ", 335, 10);
    }
  }

  // Ambil snapshot data secara thread-safe
  NodeData n1Snap;
  NodeData n2Snap;
  Node3Data n3Snap;
  float localPhSnap = NAN;

  if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(40)) == pdTRUE) {
    n1Snap = node1;
    n2Snap = node2;
    n3Snap = node3;
    localPhSnap = gatewayPh;
    xSemaphoreGive(xDataMutex);
  } else {
    return;
  }

  char buf[40];
  tft.setTextSize(1);

  // =========================================================================
  // 1. UPDATE DATA NODE 1 (DENGAN DETEKSI OFFLINE DINAMIS BERBASIS DEEP SLEEP)
  // =========================================================================
  // Hitung timeout berdasarkan durasi deep sleep terdeteksi:
  // Toleransi = 2× sleep + 5 detik (menghindari false-offline karena jitter RF/ESP-NOW)
  unsigned long n1TimeoutMs = (unsigned long)n1Snap.sleepDurationSec * 2000UL + 5000UL;
  if (n1TimeoutMs < 15000UL) n1TimeoutMs = 15000UL; // Minimum 15s agar tidak terlalu sensitif
  bool n1IsOnline = n1Snap.active && (n1Snap.lastSeen > 0) && ((millis() - n1Snap.lastSeen) < n1TimeoutMs);
  bool n1HasEverSeen = n1Snap.lastSeen > 0 || n1Snap.active;

  // Badge status online/offline Node 1 (di sudut kanan atas Card 1)
  if (n1IsOnline) {
    tft.setTextColor(COLOR_ACCENT, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[ONLINE] DS:%2ds", n1Snap.sleepDurationSec);
  } else if (n1HasEverSeen) {
    tft.setTextColor(COLOR_RED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[OFFLINE]DS:%2ds ", n1Snap.sleepDurationSec);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[TUNGGU] DS:%2ds ", n1Snap.sleepDurationSec);
  }
  tft.drawString(buf, 115, 46);

  // Suhu Udara - Tetap tampilkan nilai terakhir meskipun sensor offline
  if (!isnan(n1Snap.temperature)) {
    tft.setTextColor(n1IsOnline ? COLOR_TEXT : COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%5.1f C     ", n1Snap.temperature);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.- C     ");
  }
  tft.drawString(buf, 115, 64);

  // Lembap Udara - Tetap tampilkan nilai terakhir
  if (!isnan(n1Snap.humidity)) {
    tft.setTextColor(n1IsOnline ? COLOR_TEXT : COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%5.1f %%     ", n1Snap.humidity);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.- %%     ");
  }
  tft.drawString(buf, 115, 82);

  // Lembap Tanah - Tetap tampilkan nilai terakhir
  if (!isnan(n1Snap.soil_moisture)) {
    tft.setTextColor(n1IsOnline ? COLOR_TEXT : COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%5.1f %%     ", n1Snap.soil_moisture);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.- %%     ");
  }
  tft.drawString(buf, 115, 100);

  // Baterai Node 1 (selalu tampilkan nilai terakhir agar bisa diagnosa)
  if (!isnan(n1Snap.battery)) {
    int pct1 = hitungPersenBaterai(n1Snap.battery);
    uint16_t bCol1 = (pct1 < 20) ? COLOR_RED : (pct1 < 50) ? COLOR_ORANGE : (n1IsOnline ? COLOR_ACCENT : COLOR_TEXT_MUTED);
    tft.setTextColor(bCol1, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%4.2f V (%2d%%)     ", n1Snap.battery, pct1);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.-- V (--%%)     ");
  }
  tft.drawString(buf, 115, 118);

  // Baris Status: tampilkan kapan terakhir diterima + tanda OFFLINE jika sudah lewat batas
  if (n1IsOnline) {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    unsigned long agoSec1 = (millis() - n1Snap.lastSeen) / 1000UL;
    snprintf(buf, sizeof(buf), "#%-4lu| %2lus lalu  ", (unsigned long)n1Snap.counter, agoSec1);
  } else if (n1Snap.lastSeen > 0) {
    tft.setTextColor(COLOR_RED, COLOR_CARD_BG);
    unsigned long agoSec1 = (millis() - n1Snap.lastSeen) / 1000UL;
    if (agoSec1 < 60)        snprintf(buf, sizeof(buf), "OFFL #%-4lu %2lus lalu", (unsigned long)n1Snap.counter, agoSec1);
    else if (agoSec1 < 3600) snprintf(buf, sizeof(buf), "OFFL #%-4lu %2lum lalu", (unsigned long)n1Snap.counter, agoSec1 / 60);
    else                     snprintf(buf, sizeof(buf), "OFFL #%-4lu %2luh lalu", (unsigned long)n1Snap.counter, agoSec1 / 3600);
  } else if (n1Snap.active) {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "#%-4lu| Memori NVS ", (unsigned long)n1Snap.counter);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "Menunggu Data... ");
  }
  tft.drawString(buf, 115, 136);

  // =========================================================================
  // 2. UPDATE DATA NODE 2 (DENGAN DETEKSI OFFLINE DINAMIS BERBASIS DEEP SLEEP)
  // =========================================================================
  unsigned long n2TimeoutMs = (unsigned long)n2Snap.sleepDurationSec * 2000UL + 5000UL;
  if (n2TimeoutMs < 15000UL) n2TimeoutMs = 15000UL;
  bool n2IsOnline = n2Snap.active && (n2Snap.lastSeen > 0) && ((millis() - n2Snap.lastSeen) < n2TimeoutMs);
  bool n2HasEverSeen = n2Snap.lastSeen > 0 || n2Snap.active;

  // Badge status online/offline Node 2
  if (n2IsOnline) {
    tft.setTextColor(COLOR_CYAN, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[ONLINE] DS:%2ds", n2Snap.sleepDurationSec);
  } else if (n2HasEverSeen) {
    tft.setTextColor(COLOR_RED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[OFFLINE]DS:%2ds ", n2Snap.sleepDurationSec);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[TUNGGU] DS:%2ds ", n2Snap.sleepDurationSec);
  }
  tft.drawString(buf, 351, 46);

  // Suhu Udara - Tetap tampilkan nilai terakhir meskipun sensor offline
  if (!isnan(n2Snap.temperature)) {
    tft.setTextColor(n2IsOnline ? COLOR_TEXT : COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%5.1f C     ", n2Snap.temperature);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.- C     ");
  }
  tft.drawString(buf, 351, 64);

  // Lembap Udara - Tetap tampilkan nilai terakhir
  if (!isnan(n2Snap.humidity)) {
    tft.setTextColor(n2IsOnline ? COLOR_TEXT : COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%5.1f %%     ", n2Snap.humidity);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.- %%     ");
  }
  tft.drawString(buf, 351, 82);

  // Lembap Tanah - Tetap tampilkan nilai terakhir
  if (!isnan(n2Snap.soil_moisture)) {
    tft.setTextColor(n2IsOnline ? COLOR_TEXT : COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%5.1f %%     ", n2Snap.soil_moisture);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.- %%     ");
  }
  tft.drawString(buf, 351, 100);

  // Baterai Node 2 (selalu tampilkan nilai terakhir)
  if (!isnan(n2Snap.battery)) {
    int pct2 = hitungPersenBaterai(n2Snap.battery);
    uint16_t bCol2 = (pct2 < 20) ? COLOR_RED : (pct2 < 50) ? COLOR_ORANGE : (n2IsOnline ? COLOR_CYAN : COLOR_TEXT_MUTED);
    tft.setTextColor(bCol2, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%4.2f V (%2d%%)     ", n2Snap.battery, pct2);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), " --.-- V (--%%)     ");
  }
  tft.drawString(buf, 351, 118);

  // Baris Status Node 2
  if (n2IsOnline) {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    unsigned long agoSec2 = (millis() - n2Snap.lastSeen) / 1000UL;
    snprintf(buf, sizeof(buf), "#%-4lu| %2lus lalu  ", (unsigned long)n2Snap.counter, agoSec2);
  } else if (n2Snap.lastSeen > 0) {
    tft.setTextColor(COLOR_RED, COLOR_CARD_BG);
    unsigned long agoSec2 = (millis() - n2Snap.lastSeen) / 1000UL;
    if (agoSec2 < 60)        snprintf(buf, sizeof(buf), "OFFL #%-4lu %2lus lalu", (unsigned long)n2Snap.counter, agoSec2);
    else if (agoSec2 < 3600) snprintf(buf, sizeof(buf), "OFFL #%-4lu %2lum lalu", (unsigned long)n2Snap.counter, agoSec2 / 60);
    else                     snprintf(buf, sizeof(buf), "OFFL #%-4lu %2luh lalu", (unsigned long)n2Snap.counter, agoSec2 / 3600);
  } else if (n2Snap.active) {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "#%-4lu| Memori NVS ", (unsigned long)n2Snap.counter);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "Menunggu Data... ");
  }
  tft.drawString(buf, 351, 136);

  // =========================================================================
  // 3. UPDATE DATA NODE 3 (GAMBUT & KARHUTLA BMKG FDRS) + OFFLINE DETEKSI
  // =========================================================================
  unsigned long n3TimeoutMs = (unsigned long)n3Snap.sleepDurationSec * 2000UL + 5000UL;
  if (n3TimeoutMs < 15000UL) n3TimeoutMs = 15000UL;
  bool n3IsOnline = n3Snap.active && (n3Snap.lastSeen > 0) && ((millis() - n3Snap.lastSeen) < n3TimeoutMs);
  bool n3HasEverSeen = n3Snap.lastSeen > 0 || n3Snap.active;

  // Badge status Node 3
  if (n3IsOnline) {
    tft.setTextColor(COLOR_ORANGE, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[ONLINE] DS:%2ds", n3Snap.sleepDurationSec);
  } else if (n3HasEverSeen) {
    tft.setTextColor(COLOR_RED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[OFFLINE]DS:%2ds ", n3Snap.sleepDurationSec);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "[TUNGGU] DS:%2ds ", n3Snap.sleepDurationSec);
  }
  tft.drawString(buf, 115, 174);

  // Suhu Tanah Gambut (DS18B20 Probe) - Tetap tampilkan nilai terakhir meskipun offline
  if (!isnan(n3Snap.soil_temp)) {
    snprintf(buf, sizeof(buf), "%5.2f C     ", n3Snap.soil_temp);
    uint16_t cColor = (n3Snap.soil_temp >= 50.0f) ? COLOR_RED :
                      (n3Snap.soil_temp >= 40.0f) ? COLOR_ORANGE :
                      (n3IsOnline ? COLOR_ACCENT : COLOR_TEXT_MUTED);
    tft.setTextColor(cColor, COLOR_CARD_BG);
    tft.drawString(buf, 115, 192);
  } else {
    snprintf(buf, sizeof(buf), " --.-- C     ");
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    tft.drawString(buf, 115, 192);
  }

  // Status Bara Smoldering
  if (!isnan(n3Snap.soil_temp) && n3Snap.soil_temp >= 50.0f) {
    tft.setTextColor(COLOR_RED, COLOR_CARD_BG);
    tft.drawString("[DARURAT BARA]   ", 115, 210);
  } else if (!isnan(n3Snap.soil_temp) && n3Snap.soil_temp >= 40.0f) {
    tft.setTextColor(COLOR_ORANGE, COLOR_CARD_BG);
    tft.drawString("[WASPADA PANAS]  ", 115, 210);
  } else if (!isnan(n3Snap.soil_temp)) {
    if (n3IsOnline) {
      tft.setTextColor(COLOR_ACCENT, COLOR_CARD_BG);
      tft.drawString("[NORMAL/AMAN]    ", 115, 210);
    } else {
      tft.setTextColor(COLOR_YELLOW, COLOR_CARD_BG);
      tft.drawString("[TERAKHIR AMAN]  ", 115, 210);
    }
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    tft.drawString("[MENUNGGU PROBE] ", 115, 210);
  }

  // FWI BMKG Score & Kategori
  uint8_t lvl = (n3Snap.fwi_level < 5) ? n3Snap.fwi_level : 4;
  if (!isnan(n3Snap.fwi_score)) {
    snprintf(buf, sizeof(buf), "%4.1f [%-13s]", n3Snap.fwi_score, BMKG_LEVEL_STR[lvl]);
    tft.setTextColor(BMKG_LEVEL_COLOR[lvl], COLOR_CARD_BG);
  } else {
    snprintf(buf, sizeof(buf), "--.- [STANDBY]     ");
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
  }
  tft.drawString(buf, 115, 228);

  // Sub-indeks FDRS (FFMC, DMC, DC)
  tft.setTextColor(COLOR_TEXT, COLOR_CARD_BG);
  if (!isnan(n3Snap.ffmc)) {
    snprintf(buf, sizeof(buf), "FF:%2.0f DM:%2.0f DC:%3.0f ", n3Snap.ffmc, n3Snap.dmc, n3Snap.dc);
  } else {
    snprintf(buf, sizeof(buf), "FF:-- DM:-- DC:--- ");
  }
  tft.drawString(buf, 115, 246);

  // Baterai & Counter Paket Node 3 — tampilkan persentase & voltase baterai
  if (!isnan(n3Snap.battery)) {
    int pct3 = hitungPersenBaterai(n3Snap.battery);
    uint16_t bCol3 = (pct3 < 20) ? COLOR_RED : (pct3 < 50) ? COLOR_ORANGE : (n3IsOnline ? COLOR_ACCENT : COLOR_TEXT_MUTED);
    tft.setTextColor(bCol3, COLOR_CARD_BG);

    if (!n3IsOnline && n3HasEverSeen) {
      unsigned long agoSec3 = (millis() - n3Snap.lastSeen) / 1000UL;
      if (agoSec3 < 60)        snprintf(buf, sizeof(buf), "%4.2fV(%2d%%) OFFL%2lus", n3Snap.battery, pct3, agoSec3);
      else if (agoSec3 < 3600) snprintf(buf, sizeof(buf), "%4.2fV(%2d%%) OFFL%2lum", n3Snap.battery, pct3, agoSec3 / 60);
      else                     snprintf(buf, sizeof(buf), "%4.2fV(%2d%%) OFFL%2luh", n3Snap.battery, pct3, agoSec3 / 3600);
    } else if (n3Snap.lastSeen == 0 && n3Snap.active) {
      snprintf(buf, sizeof(buf), "%4.2fV (%2d%%) #%-4lu[NVS]", n3Snap.battery, pct3, (unsigned long)n3Snap.counter);
    } else {
      snprintf(buf, sizeof(buf), "%4.2fV (%2d%%) #%-5lu", n3Snap.battery, pct3, (unsigned long)n3Snap.counter);
    }
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "--.-- V (--%%) #0    ");
  }
  tft.drawString(buf, 115, 264);

  // =========================================================================
  // 4. UPDATE WATER FLOW, SENSOR pH LOKAL, & POMPA
  // =========================================================================
  // Status Katup Aktif di Header Card 4
  const char* valveStr = (actuatorState == STATE_IRIGASI_ACTIVE) ? (is3PhaseFertCycle ? "[PRE-WET]  " : "[IRIGASI]  ") :
                         (actuatorState == STATE_FERTIGASI_ACTIVE) ? "[FERTIGASI]" :
                         (actuatorState == STATE_POST_FLUSH_ACTIVE) ? "[POST-FLUSH]" : "[STANDBY]  ";
  uint16_t valveColor  = (actuatorState == STATE_IRIGASI_ACTIVE) ? COLOR_ACCENT :
                         (actuatorState == STATE_FERTIGASI_ACTIVE) ? COLOR_YELLOW :
                         (actuatorState == STATE_POST_FLUSH_ACTIVE) ? COLOR_CYAN : COLOR_TEXT_MUTED;
  tft.setTextColor(valveColor, COLOR_CARD_BG);
  tft.drawString(valveStr, 380, 174);

  // Sensor pH Lokal (ADC GPIO 36)
  if (!isnan(localPhSnap)) {
    const char* phCond = (localPhSnap < 5.5f) ? "[S.ASAM]" :
                         (localPhSnap < 6.5f) ? "[ASAM]  " :
                         (localPhSnap <= 7.5f) ? "[IDEAL] " : "[BASA]  ";
    uint16_t phColor   = (localPhSnap >= 6.0f && localPhSnap <= 7.5f) ? COLOR_ACCENT :
                         (localPhSnap < 5.0f || localPhSnap > 8.5f) ? COLOR_RED : COLOR_YELLOW;
    snprintf(buf, sizeof(buf), "%4.2f pH %-8s ", localPhSnap, phCond);
    tft.setTextColor(phColor, COLOR_CARD_BG);
  } else {
    snprintf(buf, sizeof(buf), "--.-- pH [WAIT]   ");
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
  }
  tft.drawString(buf, 351, 192);

  // Debit Aliran Air (YF-S201)
  snprintf(buf, sizeof(buf), "%5.2f L/min     ", flowRateLpm);
  tft.setTextColor((flowRateLpm > 0.05f) ? COLOR_CYAN : COLOR_TEXT, COLOR_CARD_BG);
  tft.drawString(buf, 351, 210);

  // Volume Sesi Irigasi/Fertigasi
  snprintf(buf, sizeof(buf), "%5.2f Liter     ", sessionWaterLiters);
  tft.setTextColor(COLOR_TEXT, COLOR_CARD_BG);
  tft.drawString(buf, 351, 228);

  // Total Air Terakumulasi
  snprintf(buf, sizeof(buf), "%5.2f Liter     ", totalWaterLiters);
  tft.setTextColor(COLOR_TEXT, COLOR_CARD_BG);
  tft.drawString(buf, 351, 246);

  // Status Pompa Relay
  if (statusPompa) {
    tft.setTextColor(COLOR_ACCENT, COLOR_CARD_BG);
    tft.drawString("ON (RELAY AKTIF) ", 351, 264);
  } else {
    tft.setTextColor(COLOR_TEXT_MUTED, COLOR_CARD_BG);
    tft.drawString("OFF (STANDBY)    ", 351, 264);
  }

  // =========================================================================
  // 5. UPDATE STATUS BAR FOOTER
  // =========================================================================
  tft.setTextColor(COLOR_TEXT_MUTED, 0x0000);
  char footer[96];
  int tH = -1, tM = -1, tS = -1;
  bool tValid = getLocalTimeSafe(tH, tM, tS);
  if (tValid) {
    snprintf(footer, sizeof(footer),
             "%02d:%02d WIB | %s | Skor:%4.1f | VPD:%4.2f | Pkt:%-5lu   ",
             tH, tM, faseVegetatif ? "VEG" : "GEN", skorTotal, vpdAgri, totalPacketsReceived);
  } else {
    unsigned long sec = millis() / 1000UL;
    snprintf(footer, sizeof(footer),
             "Up:%02luh%02lum | %s | Skor:%4.1f | VPD:%4.2f | Pkt:%-5lu   ",
             sec / 3600UL, (sec % 3600UL) / 60UL, faseVegetatif ? "VEG" : "GEN", skorTotal, vpdAgri, totalPacketsReceived);
  }
  tft.drawString(footer, 10, 303);
}

// =====================================================
// KONFIGURASI WIFI DINAMIS DENGAN AUTO-ROLLBACK
// =====================================================
void applyNewWiFi(const String& newSsid, const String& newPass) {
  Serial.println(F("\n================================================="));
  Serial.printf("[WIFI CFG] Menerima SSID baru: %s\n", newSsid.c_str());
  Serial.println(F("           Menguji koneksi WiFi baru..."));
  Serial.println(F("================================================="));

  String prevSsid = currentSsid;
  String prevPass = currentPass;

  // Kirim respon sementara ke dashboard web
  safeMqttPublish("sproutai/wifi/status", "{\"status\":\"CONNECTING\",\"message\":\"Sedang menguji koneksi WiFi baru...\"}", false);
  vTaskDelay(pdMS_TO_TICKS(300));

  WiFi.disconnect();
  vTaskDelay(pdMS_TO_TICKS(600));

  WiFi.begin(newSsid.c_str(), newPass.c_str());

  unsigned long startTry = millis();
  bool connected = false;
  while (millis() - startTry < 12000UL) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println();

  if (connected) {
    Serial.printf("[WIFI CFG] SUKSES! Terhubung ke SSID '%s' (IP: %s)\n", newSsid.c_str(), WiFi.localIP().toString().c_str());
    Serial.println(F("[WIFI CFG] Menyimpan konfigurasi baru ke Flash NVS permanen..."));

    currentSsid = newSsid;
    currentPass = newPass;

    if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      prefs.begin("teranova", false);
      prefs.putString("ssid", currentSsid);
      prefs.putString("pass", currentPass);
      prefs.end();
      xSemaphoreGive(xPrefsMutex);
    }

    // Reconnect ke MQTT
    ensureMqttConnection();

    String resp = "{\"status\":\"SUCCESS\",\"message\":\"WiFi berhasil terhubung & tersimpan permanen!\",\"ssid\":\"" + currentSsid + "\"}";
    safeMqttPublish("sproutai/wifi/status", resp.c_str(), true);
  } else {
    Serial.printf("[WIFI CFG] GAGAL terhubung ke SSID '%s'! Mengembalikan ke SSID sebelumnya '%s'...\n", newSsid.c_str(), prevSsid.c_str());

    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));
    WiFi.begin(prevSsid.c_str(), prevPass.c_str());

    unsigned long startRevert = millis();
    while (millis() - startRevert < 12000UL) {
      if (WiFi.status() == WL_CONNECTED) break;
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    ensureMqttConnection();

    String resp = "{\"status\":\"FAILED_REVERTED\",\"error\":\"Gagal terhubung (salah SSID/password). ESP32 kembali ke WiFi sebelumnya.\",\"ssid\":\"" + prevSsid + "\"}";
    safeMqttPublish("sproutai/wifi/status", resp.c_str(), true);
    Serial.println(F("[WIFI CFG] Rollback selesai. Gateway kembali online di WiFi lama."));
  }
}

// =====================================================
// MQTT CALLBACK (Perintah dari Web Dashboard)
// =====================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String strTopic = String(topic);
  String strPayload = "";
  for (unsigned int i = 0; i < length; i++) {
    strPayload += (char)payload[i];
  }
  strPayload.trim();

  // KRUSIAL: Abaikan pesan status atau telemetry agar tidak terjadi feedback loop tak terhingga.
  if (strTopic.indexOf("/status") >= 0 || strTopic.indexOf("/sensor/") >= 0) {
    return;
  }

  // CETAK SETIAP PERINTAH DARI WEB KE SERIAL MONITOR
  Serial.println(F("\n================================================="));
  Serial.printf("[RX <- WEB] Topik   : %s\n", topic);
  Serial.printf("[RX <- WEB] Perintah: %s\n", strPayload.c_str());
  Serial.println(F("================================================="));

  // 1. Mode Sistem (Auto / Manual) — Simpan ke Flash NVS
  if (strTopic.endsWith("/mode/cmd") || strTopic.indexOf("mode") >= 0) {
    if (strPayload.equalsIgnoreCase("auto") || strPayload.equalsIgnoreCase("otomatis") || strPayload.indexOf("\"auto\"") >= 0) {
      modeSistem = "OTOMATIS";
      if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        prefs.begin("teranova", false);
        prefs.putString("mode", modeSistem);
        prefs.end();
        xSemaphoreGive(xPrefsMutex);
      }
      Serial.println(F("[MODE] Tersimpan di NVS: OTOMATIS"));
      updateIndicatorLamps();
      publishActuatorMqtt();
    } else if (strPayload.equalsIgnoreCase("manual") || strPayload.indexOf("\"manual\"") >= 0) {
      modeSistem = "MANUAL";
      if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        prefs.begin("teranova", false);
        prefs.putString("mode", modeSistem);
        prefs.end();
        xSemaphoreGive(xPrefsMutex);
      }
      Serial.println(F("[MODE] Tersimpan di NVS: MANUAL"));
      updateIndicatorLamps();
      publishActuatorMqtt();
    }
    return;
  }

  // 2. Kontrol Pompa / Irigasi
  if (strTopic.endsWith("/irigasi/cmd") || strTopic.endsWith("/pompa/cmd") || strTopic.indexOf("irigasi") >= 0 || strTopic.indexOf("pompa") >= 0) {
    if (strPayload.equalsIgnoreCase("on") || strPayload.equalsIgnoreCase("1") || strPayload.equalsIgnoreCase("true") || strPayload.indexOf("\"on\"") >= 0 || strPayload.indexOf("pump_on") >= 0) {
      float target = modeSistem.equalsIgnoreCase("MANUAL") ? 999.0f : 1.5f;
      startIrigasi(target, false, 0.0f, false);
      Serial.println(F("[AKTUATOR] Irigasi diaktifkan"));
    } else if (strPayload.equalsIgnoreCase("off") || strPayload.equalsIgnoreCase("0") || strPayload.equalsIgnoreCase("false") || strPayload.indexOf("\"off\"") >= 0 || strPayload.indexOf("pump_off") >= 0) {
      stopAllActuators("CMD_OFF_IRIGASI");
      Serial.println(F("[AKTUATOR] Irigasi dimatikan"));
    }
    return;
  }

  // 3. Kontrol Fertigasi
  if (strTopic.endsWith("/fertigasi/cmd") || strTopic.indexOf("fertigasi") >= 0) {
    if (strPayload.equalsIgnoreCase("on") || strPayload.equalsIgnoreCase("1") || strPayload.equalsIgnoreCase("true") || strPayload.indexOf("\"on\"") >= 0) {
      float target = modeSistem.equalsIgnoreCase("MANUAL") ? 999.0f : 1.0f;
      startFertigasi(target, false);
      Serial.println(F("[AKTUATOR] Fertigasi diaktifkan"));
    } else if (strPayload.equalsIgnoreCase("off") || strPayload.equalsIgnoreCase("0") || strPayload.equalsIgnoreCase("false") || strPayload.indexOf("\"off\"") >= 0) {
      stopAllActuators("CMD_OFF_FERTIGASI");
      Serial.println(F("[AKTUATOR] Fertigasi dimatikan"));
    }
    return;
  }

  // 4. Konfigurasi WiFi Baru dari Web Dashboard
  if (strTopic.endsWith("/wifi/cmd") || strTopic.indexOf("wifi") >= 0) {
    JsonDocument wifiDoc;
    DeserializationError err = deserializeJson(wifiDoc, strPayload);
    if (!err) {
      String newS = wifiDoc["ssid"] | "";
      String newP = wifiDoc["password"] | "";
      if (newS.length() > 0) {
        applyNewWiFi(newS, newP);
      } else {
        Serial.println(F("[WIFI CFG] Error: SSID kosong!"));
      }
    } else {
      Serial.println(F("[WIFI CFG] Error: JSON payload tidak valid"));
    }
    return;
  }

  // 5. AI Action
  if (strTopic.startsWith("sproutai/ai/") || strTopic.indexOf("ai/action") >= 0) {
    if (strPayload.indexOf("\"pump\":\"ON\"") >= 0 || strPayload.indexOf("\"pump\":\"on\"") >= 0 || strPayload.indexOf("pump_on") >= 0) {
      startIrigasi(0.5f, false, 0.0f, false);
    } else if (strPayload.indexOf("\"pump\":\"OFF\"") >= 0 || strPayload.indexOf("\"pump\":\"off\"") >= 0 || strPayload.indexOf("pump_off") >= 0) {
      stopAllActuators("CMD_AI_OFF");
    }
    return;
  }

  // 6. Data Cuaca BMKG Eksternal (Kecepatan Angin untuk FWI ISI)
  if (strTopic.indexOf("weather/wind") >= 0 || strTopic.indexOf("angin") >= 0) {
    float w = strPayload.toFloat();
    if (w >= 0.0f) {
      externalWindKmh = w;
    }
    return;
  }

  // 7. Pengaturan Lengkap dari Web Dashboard (sproutai/settings/cmd)
  if (strTopic.endsWith("/settings/cmd") || strTopic.indexOf("settings") >= 0) {
    JsonDocument setDoc;
    DeserializationError err = deserializeJson(setDoc, strPayload);
    if (!err) {
      bool changed = false;

      // Fase Tanaman ("vegetatif" / "generatif" / boolean)
      if (setDoc["plant_phase"].is<const char*>()) {
        const char* pp = setDoc["plant_phase"].as<const char*>();
        bool newVeg = (strcasecmp(pp, "generatif") != 0 && strcasecmp(pp, "generative") != 0);
        if (newVeg != faseVegetatif) {
          faseVegetatif = newVeg;
          updateAmbangFase();
          changed = true;
          Serial.printf("[CFG] Fase Tanaman diubah: %s\n", faseVegetatif ? "VEGETATIF" : "GENERATIF");
        }
      } else if (setDoc["pp"].is<int>()) {
        bool newVeg = (setDoc["pp"].as<int>() != 1);
        if (newVeg != faseVegetatif) {
          faseVegetatif = newVeg;
          updateAmbangFase();
          changed = true;
        }
      }

      // Ambang Kelembapan Tanah (Kritis, Bawah, Atas)
      if (setDoc["soil_threshold_critical"].is<float>()) {
        ambangKritis = setDoc["soil_threshold_critical"].as<float>();
        changed = true;
      } else if (setDoc["tk"].is<float>()) {
        ambangKritis = setDoc["tk"].as<float>();
        changed = true;
      }

      if (setDoc["soil_threshold_low"].is<float>()) {
        ambangBawah = setDoc["soil_threshold_low"].as<float>();
        changed = true;
      }
      if (setDoc["soil_threshold_high"].is<float>()) {
        ambangAtas = setDoc["soil_threshold_high"].as<float>();
        changed = true;
      }

      // Ambang Batas Suhu & Kelembapan Udara Lingkungan
      if (setDoc["temp_threshold_low"].is<float>()) {
        ambangTempLow = setDoc["temp_threshold_low"].as<float>();
        changed = true;
      }
      if (setDoc["temp_threshold_high"].is<float>()) {
        ambangTempHigh = setDoc["temp_threshold_high"].as<float>();
        changed = true;
      }
      if (setDoc["humidity_threshold_low"].is<float>()) {
        ambangHumLow = setDoc["humidity_threshold_low"].as<float>();
        changed = true;
      }
      if (setDoc["humidity_threshold_high"].is<float>()) {
        ambangHumHigh = setDoc["humidity_threshold_high"].as<float>();
        changed = true;
      }

      // Jadwal Penyiraman (waktu HH:MM, durasi, status aktif)
      const char* rawWt = nullptr;
      if (setDoc["watering_time"].is<const char*>()) rawWt = setDoc["watering_time"].as<const char*>();
      else if (setDoc["wt"].is<const char*>()) rawWt = setDoc["wt"].as<const char*>();

      if (rawWt && strlen(rawWt) >= 4) {
        int jh = 0, jm = 0;
        if (sscanf(rawWt, "%d:%d", &jh, &jm) == 2) {
          if (jh >= 0 && jh <= 23 && jm >= 0 && jm <= 59) {
            if (jadwalJam != jh || jadwalMenit != jm) {
              jadwalJam = jh;
              jadwalMenit = jm;
              lastWateredDay = -1;
              lastWateredHour = -1;
              lastWateredMinute = -1;
              changed = true;
              Serial.printf("[CFG] Jadwal diatur ke: %02d:%02d\n", jadwalJam, jadwalMenit);
            }
          }
        }
      }

      if (setDoc["watering_duration"].is<int>()) {
        int wd = setDoc["watering_duration"].as<int>();
        if (jadwalPenyiramanDurasiDetik != wd) {
          jadwalPenyiramanDurasiDetik = wd;
          changed = true;
        }
      } else if (setDoc["wd"].is<int>()) {
        int wd = setDoc["wd"].as<int>();
        if (jadwalPenyiramanDurasiDetik != wd) {
          jadwalPenyiramanDurasiDetik = wd;
          changed = true;
        }
      }

      if (setDoc["schedule_enabled"].is<bool>()) {
        bool en = setDoc["schedule_enabled"].as<bool>();
        if (jadwalPenyiramanAktif != en) {
          jadwalPenyiramanAktif = en;
          lastWateredDay = -1;
          changed = true;
        }
      } else if (setDoc["watering_enabled"].is<bool>()) {
        bool en = setDoc["watering_enabled"].as<bool>();
        if (jadwalPenyiramanAktif != en) {
          jadwalPenyiramanAktif = en;
          lastWateredDay = -1;
          changed = true;
        }
      } else if (setDoc["se"].is<bool>()) {
        bool en = setDoc["se"].as<bool>();
        if (jadwalPenyiramanAktif != en) {
          jadwalPenyiramanAktif = en;
          lastWateredDay = -1;
          changed = true;
        }
      }

      // Peluang Hujan BMKG
      if (setDoc["weather_rain_chance"].is<float>()) {
        peluangHujan = setDoc["weather_rain_chance"].as<float>();
        changed = true;
      } else if (setDoc["rain"].is<float>()) {
        peluangHujan = setDoc["rain"].as<float>();
        changed = true;
      } else if (setDoc["rc"].is<float>()) {
        peluangHujan = setDoc["rc"].as<float>();
        changed = true;
      }

      // Mode Otomatis / Manual jika disertakan
      if (setDoc["auto_mode"].is<bool>()) {
        modeSistem = setDoc["auto_mode"].as<bool>() ? "OTOMATIS" : "MANUAL";
        changed = true;
        updateIndicatorLamps();
      }

      // Interval Hari Fertigasi (Pemupukan)
      if (setDoc["fertigation_interval"].is<int>()) {
        int fi = setDoc["fertigation_interval"].as<int>();
        if (fi >= 1 && fi <= 14) {
          intervalHariFertigasi = fi;
          changed = true;
          Serial.printf("[CFG] Interval Fertigasi diatur ke: %d hari sekali\n", intervalHariFertigasi);
        }
      } else if (setDoc["fert_interval"].is<int>()) {
        int fi = setDoc["fert_interval"].as<int>();
        if (fi >= 1 && fi <= 14) {
          intervalHariFertigasi = fi;
          changed = true;
          Serial.printf("[CFG] Interval Fertigasi diatur ke: %d hari sekali\n", intervalHariFertigasi);
        }
      }

      if (changed) {
        saveAgriSettingsNVS();
        publishActuatorMqtt();
        publishIrrigationScoreMqtt();
        publishScheduleMqtt();
        Serial.println(F("[CFG] Pengaturan irigasi hortikultura & jadwal berhasil diperbarui & disimpan di NVS."));
      }
    }
    return;
  }

  // 8. Pengaturan Khusus Jadwal (sproutai/schedule/cmd)
  if (strTopic.endsWith("/schedule/cmd") || strTopic.indexOf("schedule") >= 0) {
    JsonDocument schedDoc;
    DeserializationError err = deserializeJson(schedDoc, strPayload);
    if (!err) {
      bool changed = false;
      const char* rawWt = nullptr;
      if (schedDoc["watering_time"].is<const char*>()) rawWt = schedDoc["watering_time"].as<const char*>();
      else if (schedDoc["wt"].is<const char*>()) rawWt = schedDoc["wt"].as<const char*>();

      if (rawWt && strlen(rawWt) >= 4) {
        int jh = 0, jm = 0;
        if (sscanf(rawWt, "%d:%d", &jh, &jm) == 2) {
          if (jh >= 0 && jh <= 23 && jm >= 0 && jm <= 59) {
            if (jadwalJam != jh || jadwalMenit != jm) {
              jadwalJam = jh;
              jadwalMenit = jm;
              lastWateredDay = -1;
              lastWateredHour = -1;
              lastWateredMinute = -1;
              changed = true;
            }
          }
        }
      }

      if (schedDoc["schedule_enabled"].is<bool>()) {
        bool en = schedDoc["schedule_enabled"].as<bool>();
        if (jadwalPenyiramanAktif != en) {
          jadwalPenyiramanAktif = en;
          lastWateredDay = -1;
          changed = true;
        }
      } else if (schedDoc["watering_enabled"].is<bool>()) {
        bool en = schedDoc["watering_enabled"].as<bool>();
        if (jadwalPenyiramanAktif != en) {
          jadwalPenyiramanAktif = en;
          lastWateredDay = -1;
          changed = true;
        }
      } else if (schedDoc["se"].is<bool>()) {
        bool en = schedDoc["se"].as<bool>();
        if (jadwalPenyiramanAktif != en) {
          jadwalPenyiramanAktif = en;
          lastWateredDay = -1;
          changed = true;
        }
      }

      if (schedDoc["watering_duration"].is<int>()) {
        int wd = schedDoc["watering_duration"].as<int>();
        if (jadwalPenyiramanDurasiDetik != wd) {
          jadwalPenyiramanDurasiDetik = wd;
          changed = true;
        }
      } else if (schedDoc["wd"].is<int>()) {
        int wd = schedDoc["wd"].as<int>();
        if (jadwalPenyiramanDurasiDetik != wd) {
          jadwalPenyiramanDurasiDetik = wd;
          changed = true;
        }
      }

      if (changed) {
        saveAgriSettingsNVS();
        publishActuatorMqtt();
        publishIrrigationScoreMqtt();
        publishScheduleMqtt();
        Serial.printf("[CFG] Jadwal diperbarui: %02d:%02d (Aktif: %s, Durasi: %ds)\n",
                      jadwalJam, jadwalMenit, jadwalPenyiramanAktif ? "YA" : "TIDAK", jadwalPenyiramanDurasiDetik);
      }
    }
    return;
  }

  // 9. Sinkronisasi Waktu Manual / Epoch dari Browser (sproutai/time/cmd)
  if (strTopic.endsWith("/time/cmd") || strTopic.indexOf("time") >= 0) {
    JsonDocument timeDoc;
    if (!deserializeJson(timeDoc, strPayload)) {
      if (timeDoc["epoch"].is<long long>()) {
        time_t ep = (time_t)timeDoc["epoch"].as<long long>();
        struct timeval tv = { .tv_sec = ep, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        Serial.printf("[TIME] Jam RTC ESP32 berhasil disinkronkan dari Web: Epoch %ld\n", (long)ep);
        publishScheduleMqtt();
      }
    }
    return;
  }
}

// =========================================================================
// HELPER PARSER JSON SENSOR:
// - Jika bernilai numerik (termasuk 0 atau 0.0), mengisi outVal dengan angka tersebut (return true)
// - Jika bernilai null / "null" (sensor tidak terdeteksi), mengisi outVal dengan NAN (return true)
// - Jika key tidak ada sama sekali di JSON, outVal TIDAK diubah (return false)
// =========================================================================
int parseSensorField(const JsonDocument& doc, const char* key, float& outVal) {
  if (!key) return -1;
  JsonVariantConst v = doc[key];
  if (!v.is<JsonVariantConst>()) return -1; // Key tidak ditemukan di dalam JSON

  if (v.isNull()) {
    outVal = NAN;
    return 0; // Eksplisit null di payload: sensor lepas / tidak terdeteksi
  }

  if (v.is<float>() || v.is<int>() || v.is<double>() || v.is<long>()) {
    outVal = v.as<float>();
    return 1; // Nilai numerik valid (termasuk nilai 0.0)
  }

  if (v.is<const char*>()) {
    const char* str = v.as<const char*>();
    if (strcasecmp(str, "null") == 0 || strcasecmp(str, "nan") == 0) {
      outVal = NAN;
      return 0;
    }
    outVal = (float)atof(str);
    return 1;
  }

  return -1;
}

bool updateSensorFloat(const JsonDocument& doc, const char* k1, const char* k2, const char* k3, const char* k4, float& outVal) {
  const char* keys[4] = {k1, k2, k3, k4};
  for (int i = 0; i < 4; i++) {
    if (!keys[i]) continue;
    int res = parseSensorField(doc, keys[i], outVal);
    if (res >= 0) return true; // Ditemukan (baik angka maupun null)
  }
  return false;
}

bool updateSensorFloat(const JsonDocument& doc, const char* k1, const char* k2, const char* k3, float& outVal) {
  return updateSensorFloat(doc, k1, k2, k3, nullptr, outVal);
}

bool updateSensorFloat(const JsonDocument& doc, const char* k1, const char* k2, float& outVal) {
  return updateSensorFloat(doc, k1, k2, nullptr, nullptr, outVal);
}

bool updateSensorFloat(const JsonDocument& doc, const char* k1, float& outVal) {
  return updateSensorFloat(doc, k1, nullptr, nullptr, nullptr, outVal);
}

bool extractUint(const JsonDocument& doc, const char* k1, uint32_t& outVal) {
  if (k1 && !doc[k1].isNull()) {
    JsonVariantConst v = doc[k1];
    if (v.is<uint32_t>() || v.is<int>() || v.is<long>() || v.is<float>()) {
      outVal = v.as<uint32_t>();
      return true;
    } else if (v.is<const char*>()) {
      outVal = (uint32_t)strtoul(v.as<const char*>(), nullptr, 10);
      return true;
    }
  }
  return false;
}

bool extractUint(const JsonDocument& doc, const char* k1, const char* k2, uint32_t& outVal) {
  return extractUint(doc, k1, outVal) || extractUint(doc, k2, outVal);
}

bool extractUint(const JsonDocument& doc, const char* k1, const char* k2, const char* k3, uint32_t& outVal) {
  return extractUint(doc, k1, outVal) || extractUint(doc, k2, outVal) || extractUint(doc, k3, outVal);
}

// =========================================================================
// PENYIMPANAN TELEMETRI PERMANEN (NVS FLASH) UNTUK REBOOT PERSISTENCE
// Saat ESP32 mati lalu hidup lagi, layar LCD langsung memulihkan data terakhir
// =========================================================================
void saveTelemetryNVS(int nodeTarget) {
  if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(80)) == pdTRUE) {
    if (prefs.begin("tn_telem", false)) {
      if (nodeTarget == 1 || nodeTarget == 0) {
        prefs.putBool("n1_valid", true);
        prefs.putFloat("n1_t", node1.temperature);
        prefs.putFloat("n1_h", node1.humidity);
        prefs.putFloat("n1_sm", node1.soil_moisture);
        prefs.putFloat("n1_bat", node1.battery);
        prefs.putUInt("n1_cnt", node1.counter);
      }
      if (nodeTarget == 2 || nodeTarget == 0) {
        prefs.putBool("n2_valid", true);
        prefs.putFloat("n2_t", node2.temperature);
        prefs.putFloat("n2_h", node2.humidity);
        prefs.putFloat("n2_sm", node2.soil_moisture);
        prefs.putFloat("n2_bat", node2.battery);
        prefs.putUInt("n2_cnt", node2.counter);
      }
      if (nodeTarget == 3 || nodeTarget == 0) {
        prefs.putBool("n3_valid", true);
        prefs.putFloat("n3_st", node3.soil_temp);
        prefs.putFloat("n3_t", node3.temperature);
        prefs.putFloat("n3_h", node3.humidity);
        prefs.putFloat("n3_bat", node3.battery);
        prefs.putUInt("n3_cnt", node3.counter);
        prefs.putFloat("n3_fwi", node3.fwi_score);
        prefs.putUChar("n3_lvl", node3.fwi_level);
        prefs.putFloat("n3_ffmc", node3.ffmc);
        prefs.putFloat("n3_dmc", node3.dmc);
        prefs.putFloat("n3_dc", node3.dc);
      }
      if (nodeTarget == 4 || nodeTarget == 0) {
        prefs.putFloat("gw_ph", gatewayPh);
        prefs.putFloat("tot_water", totalWaterLiters);
      }
      prefs.end();
    }
    xSemaphoreGive(xPrefsMutex);
  }
}

void loadTelemetryNVS() {
  if (xPrefsMutex != NULL && xSemaphoreTake(xPrefsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (prefs.begin("tn_telem", true)) { // Read-only
      if (prefs.getBool("n1_valid", false)) {
        node1.temperature   = prefs.getFloat("n1_t", NAN);
        node1.humidity      = prefs.getFloat("n1_h", NAN);
        node1.soil_moisture = prefs.getFloat("n1_sm", NAN);
        node1.battery       = prefs.getFloat("n1_bat", NAN);
        node1.counter       = prefs.getUInt("n1_cnt", 0);
        node1.active        = true;
        node1.lastSeen      = 0; // 0 = restored from NVS (belum ada paket live baru)
        Serial.printf("[NVS RESTORE] Node 1: Suhu=%.1f C | Tanah=%.1f %% | Bat=%.2f V | Pkt=%lu\n",
                      node1.temperature, node1.soil_moisture, node1.battery, (unsigned long)node1.counter);
      }

      if (prefs.getBool("n2_valid", false)) {
        node2.temperature   = prefs.getFloat("n2_t", NAN);
        node2.humidity      = prefs.getFloat("n2_h", NAN);
        node2.soil_moisture = prefs.getFloat("n2_sm", NAN);
        node2.battery       = prefs.getFloat("n2_bat", NAN);
        node2.counter       = prefs.getUInt("n2_cnt", 0);
        node2.active        = true;
        node2.lastSeen      = 0;
        Serial.printf("[NVS RESTORE] Node 2: Suhu=%.1f C | Tanah=%.1f %% | Bat=%.2f V | Pkt=%lu\n",
                      node2.temperature, node2.soil_moisture, node2.battery, (unsigned long)node2.counter);
      }

      if (prefs.getBool("n3_valid", false)) {
        node3.soil_temp     = prefs.getFloat("n3_st", NAN);
        node3.temperature   = prefs.getFloat("n3_t", NAN);
        node3.humidity      = prefs.getFloat("n3_h", NAN);
        node3.battery       = prefs.getFloat("n3_bat", NAN);
        node3.counter       = prefs.getUInt("n3_cnt", 0);
        node3.fwi_score     = prefs.getFloat("n3_fwi", NAN);
        node3.fwi_level     = prefs.getUChar("n3_lvl", 0);
        node3.ffmc          = prefs.getFloat("n3_ffmc", NAN);
        node3.dmc           = prefs.getFloat("n3_dmc", NAN);
        node3.dc            = prefs.getFloat("n3_dc", NAN);
        node3.active        = true;
        node3.lastSeen      = 0;
        Serial.printf("[NVS RESTORE] Node 3: T_Gambut=%.2f C | FWI=%.2f | Bat=%.2f V | Pkt=%lu\n",
                      node3.soil_temp, node3.fwi_score, node3.battery, (unsigned long)node3.counter);
      }

      float restoredPh = prefs.getFloat("gw_ph", NAN);
      if (!isnan(restoredPh)) gatewayPh = restoredPh;

      float restoredWater = prefs.getFloat("tot_water", 0.0f);
      if (restoredWater > 0.0f) totalWaterLiters = restoredWater;

      prefs.end();
    }
    xSemaphoreGive(xPrefsMutex);
  }
}

static String serial2Buffer = "";
static String serialUsbBuffer = "";

void processIncomingJson(const String& cleanJson, const char* source) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, cleanJson);

  if (!error) {
    totalPacketsReceived++;

    int nodeId = 0;
    if (doc["node_id"].is<int>())        nodeId = doc["node_id"].as<int>();
    else if (doc["nodeID"].is<int>())    nodeId = doc["nodeID"].as<int>();
    else if (doc["id"].is<int>())        nodeId = doc["id"].as<int>();
    else if (doc["node"].is<int>())      nodeId = doc["node"].as<int>();
    else nodeId = 1; // Default fallback

    Serial.printf("[RX <- %s] Node %d: %s\n", source, nodeId, cleanJson.c_str());

    int pendingPublishNode = 0;
    Node3Data n3Copy;

    if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint32_t u = 0;

      // Helper lambda: deteksi dan perbarui estimasi durasi deep sleep
      // berdasarkan selisih waktu antar paket (hanya jika interval masuk range 3-120 detik)
      auto updateSleepEstimate = [](unsigned long &prevSeen, unsigned long lastSeen, uint16_t &sleepDurationSec) {
        if (prevSeen > 0 && lastSeen > prevSeen) {
          uint32_t intervalMs = (uint32_t)(lastSeen - prevSeen);
          uint32_t intervalSec = intervalMs / 1000UL;
          if (intervalSec >= 3 && intervalSec <= 120) {
            // Kuantisasi ke slot terdekat: 5, 10, 20, 30 s (atau nilai apa pun yang terukur)
            const uint16_t slots[] = {5, 10, 20, 30};
            uint16_t bestSlot = intervalSec; // fallback: nilai terukur
            uint32_t minDiff = 0xFFFF;
            for (uint8_t i = 0; i < 4; i++) {
              uint32_t diff = (intervalSec > slots[i]) ? (intervalSec - slots[i]) : (slots[i] - intervalSec);
              if (diff < minDiff) { minDiff = diff; bestSlot = slots[i]; }
            }
            // Hanya update jika penyimpangan < 40% dari nilai terukur (menghindari noise spike)
            if (minDiff <= (intervalSec / 3 + 2)) {
              sleepDurationSec = bestSlot;
            }
          }
        }
        prevSeen = lastSeen;
      };

      // -------------------------------------------------------------
      // 1. DATA NODE 1 (HORTIKULTURA A)
      // -------------------------------------------------------------
      if (nodeId == 1) {
        node1.active = true;
        unsigned long nowMs = millis();
        updateSleepEstimate(node1.prevSeen, nowMs, node1.sleepDurationSec);
        node1.lastSeen = nowMs;

        updateSensorFloat(doc, "temperature", "temp", "t", "suhu", node1.temperature);
        updateSensorFloat(doc, "humidity", "hum", "rh", "h", node1.humidity);
        updateSensorFloat(doc, "soil_moisture", "soil", "sm", "tanah", node1.soil_moisture);
        updateSensorFloat(doc, "battery", "bat", "batt", "vbat", node1.battery);
        extractUint(doc, "counter", "count", "pkt", node1.counter);
        pendingPublishNode = 1;

        saveTelemetryNVS(1);

        Serial.printf("[SYNC -> LCD N1] Suhu=%.1f C | Lembap=%.1f %% | Tanah=%.1f %% | Bat=%.2f V (%d%%) | Pkt=%lu\n",
                      node1.temperature, node1.humidity, node1.soil_moisture, node1.battery, hitungPersenBaterai(node1.battery), (unsigned long)node1.counter);

      // -------------------------------------------------------------
      // 2. DATA NODE 2 (HORTIKULTURA B)
      // -------------------------------------------------------------
      } else if (nodeId == 2) {
        node2.active = true;
        unsigned long nowMs2 = millis();
        updateSleepEstimate(node2.prevSeen, nowMs2, node2.sleepDurationSec);
        node2.lastSeen = nowMs2;

        updateSensorFloat(doc, "temperature", "temp", "t", "suhu", node2.temperature);
        updateSensorFloat(doc, "humidity", "hum", "rh", "h", node2.humidity);
        updateSensorFloat(doc, "soil_moisture", "soil", "sm", "tanah", node2.soil_moisture);
        updateSensorFloat(doc, "battery", "bat", "batt", "vbat", node2.battery);
        extractUint(doc, "counter", "count", "pkt", node2.counter);
        pendingPublishNode = 2;

        saveTelemetryNVS(2);

        Serial.printf("[SYNC -> LCD N2] Suhu=%.1f C | Lembap=%.1f %% | Tanah=%.1f %% | Bat=%.2f V (%d%%) | Pkt=%lu\n",
                      node2.temperature, node2.humidity, node2.soil_moisture, node2.battery, hitungPersenBaterai(node2.battery), (unsigned long)node2.counter);

      // -------------------------------------------------------------
      // 3. DATA NODE 3 (GAMBUT & KARHUTLA BMKG FDRS)
      // -------------------------------------------------------------
      } else if (nodeId == 3) {
        node3.active = true;
        unsigned long nowMs3 = millis();
        updateSleepEstimate(node3.prevSeen, nowMs3, node3.sleepDurationSec);
        node3.lastSeen = nowMs3;

        // Suhu Tanah Gambut probe DS18B20 murni
        if (!updateSensorFloat(doc, "soil_temp", "soil_temperature", "t_tanah", "temperature", node3.soil_temp)) {
          updateSensorFloat(doc, "temp", "suhu", node3.soil_temp);
        }

        // Baterai Node 3
        updateSensorFloat(doc, "battery", "bat", "batt", "vbat", node3.battery);

        // Counter Paket Node 3
        extractUint(doc, "counter", "count", "pkt", node3.counter);

        // Ambient Gambut (ambil dari payload jika ada, atau sinkron dari Node 1 / Node 2)
        float ambT = NAN;
        float ambRH = NAN;
        updateSensorFloat(doc, "ambient_temp", "ambient_temperature", ambT);
        updateSensorFloat(doc, "ambient_hum", "ambient_humidity", ambRH);

        if (isnan(ambT)) {
          if (node1.active && (millis() - node1.lastSeen < 60000UL) && !isnan(node1.temperature)) {
            ambT = node1.temperature;
          } else if (node2.active && (millis() - node2.lastSeen < 60000UL) && !isnan(node2.temperature)) {
            ambT = node2.temperature;
          } else if (!isnan(node1.temperature)) {
            ambT = node1.temperature;
          } else {
            ambT = 30.0f;
          }
        }

        if (isnan(ambRH)) {
          if (node1.active && (millis() - node1.lastSeen < 60000UL) && !isnan(node1.humidity)) {
            ambRH = node1.humidity;
          } else if (node2.active && (millis() - node2.lastSeen < 60000UL) && !isnan(node2.humidity)) {
            ambRH = node2.humidity;
          } else if (!isnan(node1.humidity)) {
            ambRH = node1.humidity;
          } else {
            ambRH = 65.0f;
          }
        }

        node3.temperature = ambT;
        node3.humidity    = ambRH;

        // Kalkulasi FWI BMKG
        node3.ffmc = hitungFFMC(ambT, ambRH, gwFFMC);
        node3.dmc  = hitungDMC(ambT, ambRH, gwDMC);
        node3.dc   = hitungDC(ambT, node3.soil_temp, gwDC);
        node3.isi  = hitungISI(node3.ffmc, externalWindKmh);
        node3.bui  = hitungBUI(node3.dmc, node3.dc);
        node3.fwi_score = hitungFWI(node3.isi, node3.bui);

        gwFFMC = isnan(node3.ffmc) ? gwFFMC : node3.ffmc;
        gwDMC  = isnan(node3.dmc)  ? gwDMC  : node3.dmc;
        gwDC   = isnan(node3.dc)   ? gwDC   : node3.dc;

        node3.fwi_level = tentukanFireLevelGateway(node3.fwi_score, node3.soil_temp);
        n3Copy = node3;
        pendingPublishNode = 3;

        saveTelemetryNVS(3);

        Serial.printf("[SYNC -> LCD N3] T_Gambut=%.2f C | FWI=%.2f [%s] | Bat=%.2f V (%d%%) | Pkt=%lu\n",
                      node3.soil_temp, node3.fwi_score, BMKG_LEVEL_STR[node3.fwi_level < 5 ? node3.fwi_level : 4],
                      node3.battery, hitungPersenBaterai(node3.battery), (unsigned long)node3.counter);
      }

      xSemaphoreGive(xDataMutex);
    }

    // UPDATE LCD TFT SECARA INSTAN! Menjamin nilai langsung sama persis dengan Serial Monitor saat paket tiba
    updateDynamicValues();
    lastDisplayUpdate = millis();

    // Publikasi MQTT di luar xDataMutex
    if (pendingPublishNode == 1) {
      processData(1, cleanJson);
    } else if (pendingPublishNode == 2) {
      processData(2, cleanJson);
    } else if (pendingPublishNode == 3) {
      publishNode3Mqtt(n3Copy);
    }
  } else {
    Serial.printf("[JSON ERR] Gagal deserialize dari %s: %s | Raw: %s\n", source, error.c_str(), cleanJson.c_str());
  }
}

void parseBufferStream(String& buf, const char* source) {
  int startIdx = buf.indexOf('{');
  while (startIdx >= 0) {
    int endIdx = buf.indexOf('}', startIdx);
    if (endIdx > startIdx) {
      String cleanJson = buf.substring(startIdx, endIdx + 1);
      processIncomingJson(cleanJson, source);
      buf = buf.substring(endIdx + 1);
      startIdx = buf.indexOf('{');
    } else {
      break; // Tunggu data sisa sampai kurung kurawal tutup '}'
    }
  }
  if (buf.length() > 512) buf = "";
}

// =========================================================================
// BACA STREAM SERIAL (HARDWARE SERIAL2 GPIO 16/17 + USB SERIAL MONITOR PC)
// =========================================================================
void readSerialStreams() {
  // 1. Baca dari Hardware Serial2 (ESP8266 Receiver via GPIO 16 RX / GPIO 17 TX)
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') continue;
    serial2Buffer += c;
    if (c == '\n' || c == '}') {
      parseBufferStream(serial2Buffer, "ESP-NOW/UART");
    }
  }

  // 2. Baca dari USB Serial (Serial Monitor PC untuk pengujian / simulasi langsung)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    serialUsbBuffer += c;
    if (c == '\n' || c == '}') {
      parseBufferStream(serialUsbBuffer, "USB-SERIAL");
    }
  }
}

// =====================================================
// KONEKSI WIFI & MQTT
// =====================================================
bool wifiWasConnected = false;
unsigned long lastReconnectAttempt = 0;

void ensureWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("[WiFi] Connected! IP: ");
      Serial.println(WiFi.localIP());
      secureClient.setInsecure();
      updateIndicatorLamps();
      configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com", "time.windows.com");
      Serial.println(F("[NTP] Memperbarui sinkronisasi waktu NTP WIB (UTC+7)..."));
    }
    return;
  }
  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("[WiFi] Lost. Auto-reconnecting...");
    updateIndicatorLamps();
  }
  unsigned long now = millis();
  if (now - lastReconnectAttempt >= 10000) {
    WiFi.reconnect();
    lastReconnectAttempt = now;
  }
}

static unsigned long lastMqttAttempt = 0;

void ensureMqttConnection() {
  if (WiFi.status() != WL_CONNECTED) return;

  // Batasi percobaan koneksi: jangan membebani Core 0 jika internet sedang offline
  unsigned long now = millis();
  if (now - lastMqttAttempt < 8000UL) return;
  lastMqttAttempt = now;

  if (xMqttMutex != NULL && xSemaphoreTakeRecursive(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (!mqttClient.connected()) {
      Serial.println(F("[MQTT] Mencoba koneksi ke HiveMQ Cloud..."));
      if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password, "sproutai/esp32/status", 1, true, "offline")) {
        Serial.println(F("\n[MQTT] Connected to HiveMQ Cloud!"));
        mqttClient.publish("sproutai/esp32/status", "online", true);

        mqttClient.subscribe("sproutai/pompa/cmd");
        mqttClient.subscribe("sproutai/irigasi/cmd");
        mqttClient.subscribe("sproutai/fertigasi/cmd");
        mqttClient.subscribe("sproutai/mode/cmd");
        mqttClient.subscribe("sproutai/wifi/cmd");
        mqttClient.subscribe("sproutai/settings/cmd");
        mqttClient.subscribe("sproutai/schedule/cmd");
        mqttClient.subscribe("sproutai/time/cmd");
        mqttClient.subscribe("sproutai/ai/#");
        mqttClient.subscribe("sproutai/weather/#");
        Serial.println(F("[MQTT] Berhasil subscribe seluruh perintah Web Dashboard!\n"));

        // Publikasikan status terkini langsung ke broker (dengan retained = true)
        publishActuatorMqtt();
        publishScheduleMqtt();
      } else {
        Serial.printf("[MQTT] Koneksi Cloud belum berhasil (rc=%d). ESP32 tetap melayani sensor secara LOKAL.\n", mqttClient.state());
      }
    }
    xSemaphoreGiveRecursive(xMqttMutex);
  }
}

// =========================================================================
// FREERTOS TASK: CORE 0 (NETWORKING & COMMUNICATION)
// Berjalan di Core 0: Menangani WiFi, TLS MQTT, dan Buffer Offline
// =========================================================================
void TaskCore0_Network(void *pvParameters) {
  Serial.printf("[FreeRTOS] Core 0 Task berjalan pada Core %d\n", xPortGetCoreID());

  // Lepas IDLE0 dari pengawasan WDT — TLS handshake mqttClient.connect()
  // bisa memblokir Core 0 selama beberapa detik, jika IDLE0 diawasi WDT akan abort.
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

  unsigned long lastMqttCheck = 0;
  unsigned long lastBufferFlush = 0;
  unsigned long lastStatusPing = 0;

  for (;;) {
    ensureWiFiConnection();

    if (millis() - lastMqttCheck >= 4000) {
      ensureMqttConnection();
      lastMqttCheck = millis();
    }

    if (xMqttMutex != NULL && xSemaphoreTakeRecursive(xMqttMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
      if (mqttClient.connected()) {
        mqttClient.loop();

        if (millis() - lastStatusPing >= 15000) {
          mqttClient.publish("sproutai/esp32/status", "online");
          lastStatusPing = millis();
        }
      }
      xSemaphoreGiveRecursive(xMqttMutex);
    }

    // Flush buffer offline secara berkala jika koneksi kembali online
    if (millis() - lastBufferFlush >= 20000) {
      flushBuffer();
      lastBufferFlush = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(15)); // Jeda 15ms untuk FreeRTOS scheduler Core 0
  }
}

// =====================================================
// SETUP SISTEM
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  // Muat konfigurasi tersimpan dari Flash NVS
  prefs.begin("teranova", false);
  String lastDefSsid = prefs.getString("def_ssid", "");
  String lastDefPass = prefs.getString("def_pass", "");
  // Jika kredensial di secret.h diubah oleh pengguna, sinkronkan otomatis ke Flash NVS
  if (lastDefSsid != String(WIFI_SSID) || lastDefPass != String(WIFI_PASS)) {
    currentSsid = WIFI_SSID;
    currentPass = WIFI_PASS;
    prefs.putString("ssid", currentSsid);
    prefs.putString("pass", currentPass);
    prefs.putString("def_ssid", WIFI_SSID);
    prefs.putString("def_pass", WIFI_PASS);
    Serial.println(F("[NVS] Terdeteksi perubahan kredensial baru di secret.h! NVS otomatis diperbarui."));
  } else {
    currentSsid = prefs.getString("ssid", WIFI_SSID);
    currentPass = prefs.getString("pass", WIFI_PASS);
  }
  modeSistem  = prefs.getString("mode", "MANUAL"); // Default MANUAL jika belum ada
  prefs.end();

  Serial.println("=================================================");
  Serial.printf(" TeraNova ESP32 Gateway (Mode: %s)\n", modeSistem.c_str());
  Serial.printf(" WiFi Target: %s\n", currentSsid.c_str());
  Serial.println("=================================================");

  // Inisialisasi Mutex FreeRTOS
  xDataMutex = xSemaphoreCreateMutex();
  xMqttMutex = xSemaphoreCreateRecursiveMutex();
  xBufferMutex = xSemaphoreCreateMutex();
  xPrefsMutex = xSemaphoreCreateMutex();

  // Inisialisasi Sinkronisasi Waktu Internet NTP (WIB = UTC+7)
  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
  Serial.println(F("[NTP] Sinkronisasi waktu WIB (UTC+7) diinisialisasi."));

  // Muat data telemetri terakhir dari Flash NVS (menampilkan data terakhir saat ESP32 mati lalu hidup lagi)
  loadTelemetryNVS();
  // Sinkronkan pulse counter ISR dari total air yang di-restore NVS
  // agar updateFlowSensorMetrics() tidak me-reset totalWaterLiters ke 0 saat boot
  if (totalWaterLiters > 0.0f) {
    totalFlowPulses = (uint32_t)(totalWaterLiters * FLOW_PULSES_PER_LITER);
    lastSnapPulses = totalFlowPulses;
  }
  // Inisialisasi tracker perubahan NVS setelah restore (agar tidak langsung menulis ulang nilai yang sama)
  lastSavedPh = gatewayPh;
  lastSavedWater = totalWaterLiters;

  // Muat konfigurasi irigasi lahan tidak gambut (hortikultura) dari Flash NVS
  loadAgriSettingsNVS();

  // Inisialisasi Hardware Serial2 (RX=16, TX=17)
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Inisialisasi Aktuator & Sensor Analog pH & Waterflow
  initActuatorHardware();

  // Inisialisasi TFT Display (ILI9488 SPI)
  tft.init();
  tft.setRotation(3); // 3 = Landscape Terbalik 180° (480x320)
  drawStaticDashboard();
  updateDynamicValues();

  // Konfigurasi WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(currentSsid.c_str(), currentPass.c_str());

  // Setup MQTT Client dengan timeout cepat agar tidak membekukan sistem saat offline
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(15);
  mqttClient.setBufferSize(512);
  secureClient.setInsecure();
  secureClient.setTimeout(2);
  secureClient.setHandshakeTimeout(3);
  mqttClient.setSocketTimeout(2);

  // Launch Core 0 Task untuk Jaringan & Komunikasi
  xTaskCreatePinnedToCore(
    TaskCore0_Network,      // Fungsi Task
    "TaskCore0_Network",    // Nama Task
    16384,                  // Stack Size (16 KB aman untuk TLS WiFiClientSecure + JSON)
    NULL,                   // Parameter
    1,                      // Prioritas Task
    NULL,                   // Task Handle
    0                       // Core 0 (PRO_CPU)
  );

  Serial.println("[FreeRTOS] Core 0 Task (Network/Serial) berhasil dibuat.");
  Serial.println("[FreeRTOS] Core 1 (UI, Local pH, Waterflow, UART RX) berjalan pada loop().\n");
}

// =========================================================================
// LOOP: BERJALAN DI CORE 1 (SENSING, FSM, ACTUATOR, SERIAL2 RX & TFT UI)
// =========================================================================
unsigned long lastAutoEval = 0;
unsigned long lastActPub = 0;

void loop() {
  // 1. Baca UART Serial2 (ESP8266 Receiver) & USB Serial (Serial Monitor PC) secara real-time di Core 1
  // Menjamin seluruh paket data Node 1, 2, 3 selalu diterima dan tersinkron langsung ke layar LCD TFT
  readSerialStreams();

  // 1b. Update Relay Lampu Indikator Mode (D26) & WiFi Online (D25) secara real-time
  updateIndicatorLamps();

  // 2. Pembacaan Sensor pH Lokal ESP32 (ADC GPIO 36) setiap 2 detik
  if (millis() - lastPhReadTime >= 2000UL) {
    lastPhReadTime = millis();
    float phVal = bacaSensorPhLokal();

    if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      gatewayPh = phVal;
      xSemaphoreGive(xDataMutex);
    }

    // Cetak sinkronisasi sensor lokal ke Serial Monitor agar sama persis dengan LCD
    Serial.printf("[LOCAL SENSOR] pH: %.2f | Debit: %.2f L/min | Sesi: %.2f L | Total: %.2f L\n",
                  gatewayPh, flowRateLpm, sessionWaterLiters, totalWaterLiters);

    // Update layar jika ada perubahan pH
    updateDynamicValues();
    lastDisplayUpdate = millis();

    // Publish ke MQTT jika terhubung
    if (millis() - lastPhMqttPub >= 3000UL) {
      publishPhMqtt(gatewayPh);
      lastPhMqttPub = millis();
    }
  }

  // 3. Update Metrik Water Flow Sensor
  updateFlowSensorMetrics();

  // 4. Jalankan Finite State Machine (FSM) Kontrol Irigasi & Fertigasi
  processActuatorFSM();

  // 5. Evaluasi Penjadwalan Waktu (Jadwal Penyiraman Otomatis Tepat Waktu)
  static unsigned long lastScheduleEval = 0;
  if (millis() - lastScheduleEval >= 1000UL) {
    evaluasiJadwalPenyiraman();
    lastScheduleEval = millis();
  }

  // 6. Evaluasi Otomatisasi Responsif Sensor (Irigasi Hortikultura & Rewetting Gambut Node 3)
  if (millis() - lastAutoEval >= 3000UL) {
    evaluasiAktuatorOtomatis();
    lastAutoEval = millis();
  }

  // 7. Publikasikan Status Aktuator, Skor Irigasi, & Jadwal ke MQTT jika terhubung
  unsigned long actPubInterval = (actuatorState != STATE_IDLE) ? 2000UL : 10000UL;
  if (millis() - lastActPub >= actPubInterval) {
    publishActuatorMqtt();
    publishIrrigationScoreMqtt();
    publishScheduleMqtt();
    lastActPub = millis();
  }

  // 7. Refresh Tampilan LCD TFT (Full Dashboard: Node 1, 2, 3, Water Flow, pH Sensor)
  if (millis() - lastDisplayUpdate >= 600UL) {
    updateDynamicValues();
    lastDisplayUpdate = millis();
  }

  // 8. Simpan metrik sensor lokal ke NVS Flash HANYA saat ada perubahan nilai signifikan
  //    (Optimasi: mengurangi siklus tulis Flash agar chip lebih awet)
  static unsigned long lastLocalNvsCheck = 0;
  if (millis() - lastLocalNvsCheck >= 20000UL) {
    lastLocalNvsCheck = millis();
    bool needSave = false;

    // Cek perubahan pH (> 0.1 unit dianggap signifikan)
    if (isnan(lastSavedPh) != isnan(gatewayPh)) {
      needSave = true;
    } else if (!isnan(gatewayPh) && fabs(gatewayPh - lastSavedPh) > 0.1f) {
      needSave = true;
    }

    // Cek perubahan total air (> 0.05 Liter dianggap signifikan)
    if (fabs(totalWaterLiters - lastSavedWater) > 0.05f) {
      needSave = true;
    }

    if (needSave) {
      saveTelemetryNVS(4);
      lastSavedPh = gatewayPh;
      lastSavedWater = totalWaterLiters;
      Serial.printf("[NVS OPT] Sensor lokal disimpan (pH: %.2f | Total Air: %.2f L)\n", gatewayPh, totalWaterLiters);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}
