# ESP32 Gateway — Smart Gateway, BMKG-FDRS Edge Computing & Closed-Loop Water Management

ESP32 Gateway adalah pusat kendali (*master gateway*) seluruh ekosistem **NexaGrow / SproutAI**. Gateway ini bertugas menerima data serial dari ESP8266 Receiver, memproses komputasi analitik prediksi karhutla **BMKG Fire Danger Rating System (FDRS)**, mengendalikan dua jalur aktuator (Irigasi & Fertigasi) secara *closed-loop* berbasis **sensor Water Flow YF-S201**, menampilkan visualisasi real-time pada **Layar TFT ILI9488 (480×320)**, dan meneruskan data terenkripsi (TLS/SSL port 8883) ke **HiveMQ Cloud** serta dashboard **TeranaWeb**.

---

## 1. Arsitektur Sistem Gateway

```
                 ┌──────────────┐      ┌──────────────┐      ┌──────────────┐
                 │ Sensor Point │      │ Sensor Point │      │ Sensor Point │
                 │  1 (Bedeng A)│      │  2 (Bedeng B)│      │  3 (Gambut)  │
                 │  DHT22+Soil  │      │  DHT22+Soil  │      │DHT22+Soil    │
                 │              │      │              │      │+DS18B20      │
                 └──────┬───────┘      └──────┬───────┘      └──────┬───────┘
                        │                     │                     │
                        │     ESP-NOW Ch.1 (2.4 GHz, tanpa WiFi)    │
                        └─────────────┬───────┴─────────────────────┘
                                      ▼
                         ┌────────────────────────┐
                         │  ESP8266 Receiver Hub   │
                         │  + Sensor pH PH-4502C   │
                         │  (Analog A0)            │
                         └────────────┬────────────┘
                                      │ Hardware UART (TX→RX2, RX→TX2)
                                      ▼
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                     ESP32 SMART GATEWAY (Edge Computing)               │
    │                                                                         │
    │  ┌─────────────────┐  ┌──────────────────┐  ┌───────────────────────┐  │
    │  │ JSON Parser &   │  │ BMKG-FDRS Engine │  │ Closed-Loop Actuator  │  │
    │  │ Multi-Node       │  │ Van Wagner 1987  │  │ FSM Controller        │  │
    │  │ Handler          │  │ FFMC DMC DC ISI  │  │ Irigasi + Fertigasi   │  │
    │  │                  │  │ BUI FWI          │  │ + Water Flow Sensor   │  │
    │  └─────────────────┘  └──────────────────┘  └───────────────────────┘  │
    │                                                                         │
    │  ┌──────────────────┐  ┌──────────────────────────────────────────────┐ │
    │  │ TFT ILI9488 SPI  │  │ HiveMQ Cloud (TLS 8883) + TeranaWeb        │ │
    │  │ 480×320 Landscape│  │ + Offline Circular Buffer (30 Paket)        │ │
    │  └──────────────────┘  └──────────────────────────────────────────────┘ │
    └─────────────────────────────────────────────────────────────────────────┘
                    │               │                  │
                    ▼               ▼                  ▼
            ┌─────────────┐ ┌─────────────┐   ┌─────────────┐
            │ Relay 1     │ │ Relay 2     │   │ Relay 3     │
            │ IRIGASI     │ │ FERTIGASI   │   │ LAMPU GROW  │
            │ (GPIO 25)   │ │ (GPIO 26)   │   │ (GPIO 32)   │
            └──────┬──────┘ └──────┬──────┘   └─────────────┘
                   │               │
                   └───────┬───────┘
                           ▼
                ┌─────────────────────┐
                │  Water Flow Sensor  │
                │  YF-S201 (GPIO 27)  │
                │  Interrupt RISING   │
                └─────────────────────┘
```

---

## 2. Spesifikasi Hardware & Pin Wiring

* **Mikrokontroler:** DOIT ESP32 DevKit V1 (Dual-Core Xtensa LX6, 240 MHz, 320KB SRAM, 4MB Flash)
* **Display:** LCD TFT ILI9488 SPI (Resolusi 480×320 Landscape, 3.5 inci)
* **Konektivitas:** WiFi 2.4 GHz + WiFiClientSecure (TLS/MQTTS Port 8883)
* **Broker MQTT:** HiveMQ Cloud (`a4e9379a555f47669c90f4c69b75eeda.s1.eu.hivemq.cloud`)
* **Sensor Aliran:** YF-S201 Hall-Effect Water Flow Sensor (~450 pulsa/Liter)
* **Aktuator:** 3× Relay Modul (Aktif LOW), mengendalikan solenoid valve irigasi, solenoid valve fertigasi, dan lampu grow light

### 2.1 Diagram Pengkabelan (Wiring Pinout)

| Komponen | Pin Modul | Pin ESP32 Gateway | Keterangan |
|---|---|---|---|
| **ESP8266 Receiver** | TX | **GPIO 16 (RX2)** | Penerima data Serial dari Receiver Hub |
| | RX | **GPIO 17 (TX2)** | Transmisi Serial ke Receiver Hub |
| | GND | **GND** | Ground referensi bersama (wajib) |
| **Layar TFT ILI9488** | VCC | 5V / 3.3V | Catu daya layar |
| | GND | GND | Ground |
| | MOSI / SDI | **GPIO 23** | SPI Master Out Slave In |
| | SCK / CLK | **GPIO 18** | SPI Serial Clock |
| | CS | **GPIO 15** | TFT Chip Select |
| | DC | **GPIO 2** | Data / Command Control |
| | RST | **GPIO 4** | TFT Reset Pin |
| | LED / BL | 3.3V | Backlight layar LCD |
| **Water Flow YF-S201** | Signal (Kuning) | **GPIO 27** | Input Interrupt RISING (INPUT_PULLUP) |
| | VCC (Merah) | 5V | Catu daya sensor Hall-Effect |
| | GND (Hitam) | GND | Ground |
| **Relay 1: Irigasi** | IN1 | **GPIO 25** | Solenoid valve air bersih (Aktif LOW) |
| **Relay 2: Fertigasi** | IN2 | **GPIO 26** | Solenoid valve larutan nutrisi/amelioran (Aktif LOW) |
| **Relay 3: Lampu Grow** | IN3 | **GPIO 32** | Lampu pertumbuhan (Aktif LOW) |
| **Modul Relay (Semua)** | VCC | 5V | Catu daya relay modul |
| | GND | GND | Ground |

### 2.2 Skema Pipa Sensor Water Flow (Piping Diagram)

Sensor water flow YF-S201 dipasang pada pipa utama (*main header*) **sebelum** percabangan ke dua solenoid valve. Dengan demikian, satu sensor mengukur volume air untuk **dua relay secara bergantian** (sekuensial, tidak bersamaan):

```
    [ Sumber Air / Pompa Utama ]
                 │
                 ▼
    ┌────────────────────────┐
    │ Water Flow YF-S201     │  ← Membaca pulsa setiap aliran melewati kincir
    │ (GPIO 27 Interrupt)    │
    └───────────┬────────────┘
                │  Pipa utama (main header)
         ┌──────┴──────┐
         ▼             ▼
  ┌──────────────┐ ┌──────────────┐
  │ Solenoid     │ │ Solenoid     │
  │ Valve 1      │ │ Valve 2      │
  │ IRIGASI      │ │ FERTIGASI    │
  │ (GPIO 25)    │ │ (GPIO 26)    │
  └──────┬───────┘ └──────┬───────┘
         ▼                ▼
  [ Bedeng / Lahan Gambut ]
```

> **Aturan Fundamental:** Relay 1 dan Relay 2 dijamin oleh firmware tidak akan aktif secara bersamaan (hardware & software interlock). Saat fase irigasi, hanya Relay 1 yang terbuka sehingga seluruh pulsa flow sensor terhitung sebagai volume air irigasi. Begitu target tercapai, Relay 1 ditutup, jeda 3 detik untuk stabilisasi tekanan pipa, baru kemudian Relay 2 (fertigasi) dibuka untuk mengukur volume larutan nutrisi secara terpisah.

---

## 3. Logika Prediksi Karhutla BMKG-FDRS (Node 3)

ESP32 Gateway mengeksekusi 6 rumus standar internasional Van Wagner (1987) yang menjadi acuan resmi **BMKG SPARTAN** dan Peta Peringatan Dini Kebakaran Hutan dan Lahan:

1. **FFMC (*Fine Fuel Moisture Code*):** Menghitung kekeringan bahan bakar halus serasah permukaan tanah dari parameter suhu udara (T) dan kelembaban relatif (RH) yang dibaca oleh DHT22 pada Sensor Point 3.
2. **DMC (*Duff Moisture Code*):** Menilai kadar air lapisan organik sedang gambut, dikoreksi langsung oleh sensor kelembaban tanah kapasitif Sensor Point 3 (faktor pengurang laju pengeringan jika tanah masih lembap > 50%).
3. **DC (*Drought Code*):** Indikator kekeringan lapisan dalam gambut — **parameter paling kritis untuk kebakaran gambut bawah permukaan**. Dikoreksi oleh persentase kadar air gambut dan suhu kedalaman tanah DS18B20 waterproof (smoldering fire boost saat suhu tanah ≥ 40°C).
4. **ISI (*Initial Spread Index*):** Kecepatan awal penyebaran api, menggabungkan nilai FFMC dan kecepatan angin eksternal dari BMKG (diterima via MQTT topik `sproutai/weather/wind`).
5. **BUI (*Build-Up Index*):** Total akumulasi bahan bakar kering yang siap terbakar (kombinasi DMC dan DC).
6. **FWI (*Fire Weather Index*):** Indeks final akumulatif yang menentukan tingkat keparahan kebakaran hutan dan lahan.

### 3.1 Standar Kategori Bahaya BMKG

| Level | Kategori | Rentang FWI | Warna TFT & Peta BMKG | Respon Sistem |
|:---:|---|---|---|---|
| 0 | 🔵 **RENDAH** | FWI 0 – 8 | Biru (`0x03DF`) | Aman, lahan gambut basah |
| 1 | 🟢 **SEDANG** | FWI 9 – 17 | Hijau (`0x07E0`) | Waspada, api menyebar lambat |
| 2 | 🟡 **TINGGI** | FWI 18 – 32 | Kuning (`0xFFE0`) | Siaga, kirim alert MQTT + trigger rewetting otomatis |
| 3 | 🔴 **SANGAT TINGGI** | FWI > 32 | Merah (`0xF800`) | Bahaya, trigger rewetting masif + amelioran penetral |
| 4 | 🚨 **DARURAT** | Suhu Tanah ≥ 50°C | Magenta (`0xF81F`) | *Smoldering fire* aktif bawah permukaan! Volume rewetting maksimal |

### 3.2 Data Sensor Khusus Node 3 (Karhutla Gambut)

Sensor Point 3 dilengkapi sensor tambahan dibanding Node 1 dan Node 2:

| Sensor | Fungsi | Pin di ESP8266 Node 3 |
|---|---|---|
| DHT22 | Suhu & kelembaban udara di atas lahan gambut | D2 (GPIO 4) |
| Capacitive Soil Moisture | Kadar air tanah gambut (%) | A0 (ADC) |
| DS18B20 Waterproof | Suhu tanah gambut bawah permukaan (°C) — indikasi smoldering fire | D4 (GPIO 2) |

Data dari DS18B20 dikirim melalui field `battery` pada `SensorPacket` (untuk kompatibilitas struct ESP-NOW dengan Node 1/2), lalu dipetakan menjadi `soil_temp` oleh ESP32 Gateway.

---

## 4. Sistem Kontrol Aktuator Closed-Loop & Water Flow

### 4.1 Finite State Machine (FSM) Aktuator

ESP32 Gateway menjalankan mesin-keadaan (*Finite State Machine*) dengan 4 state untuk mengeksekusi siklus penyiraman secara sekuensial dan terukur:

```
 ┌──────────┐    Trigger (Soil kering /     ┌────────────────────┐
 │  IDLE    │ ──FWI tinggi / perintah web)──▶│ IRIGASI_ACTIVE     │
 │          │                                │ Relay 1 ON         │
 └──────────┘                                │ Hitung volume air  │
      ▲                                      └─────────┬──────────┘
      │                                      Volume tercapai ▼
      │ Siklus tuntas                        ┌────────────────────┐
      │                                      │ INTER_DELAY        │
      │                                      │ Jeda 3 detik       │
      │                                      │ Stabilisasi tekanan│
      │                                      └─────────┬──────────┘
      │                                                ▼
      │                                      ┌────────────────────┐
      └──────────────────────────────────────│ FERTIGASI_ACTIVE   │
                    Volume tercapai          │ Relay 2 ON         │
                                             │ Hitung volume pupuk│
                                             └────────────────────┘
```

### 4.2 Tiga Model Kalkulasi Volume Air

#### A. Irigasi Pertanian (Node 1 & Node 2)

**Dasar:** Sensor kelembaban tanah kapasitif (Soil Moisture) + DHT22 (suhu & kelembaban udara).

* **Pemicu:** Kadar air tanah bedeng turun di bawah 45%.
* **Rumus Target Volume:**
  - Defisit kelembaban: ΔSM = 75% − Soil Moisture (%)
  - Faktor evapotranspirasi cuaca: F_evap = 1.0 + max(0, (T − 26) × 0.025) + max(0, (65 − RH) × 0.015)
  - **Volume Target Irigasi = (ΔSM / 25) × 1.5 Liter × F_evap**
  - Dibatasi dalam rentang aman **0.5 – 4.0 Liter** per siklus.

#### B. Fertigasi Nutrisi (Berdasarkan Sensor pH)

**Dasar:** Sensor pH analog PH-4502C (dibaca oleh ESP8266 Receiver Hub di pin A0).

* **Pemicu:** Penyerapan hara tanaman optimal pada pH 6.0 – 7.0 (titik ideal pH = 6.5). Fertigasi dipicu jika pH < 5.8 atau pH > 7.3.
* **Rumus Target Volume:**
  - Deviasi pH: ΔpH = |pH − 6.5|
  - **Volume Target Fertigasi = 0.8 Liter × (1.0 + (ΔpH × 0.35))**
  - Dibatasi dalam rentang aman **0.3 – 2.5 Liter** per siklus.

#### C. Gabungan Irigasi Rewetting & Fertigasi Amelioran Gambut (Node 3 — Karhutla)

**Dasar:** Gabungan data FWI BMKG, sensor kelembaban gambut, suhu tanah DS18B20, dan sensor pH.

Lahan gambut yang terancam kebakaran memiliki dua masalah sekaligus: **ancaman kebakaran bawah permukaan (smoldering fire)** yang memerlukan injeksi air masif, serta **keasaman ekstrem** (pH < 4.5 akibat dekomposisi organik gambut kering) yang memerlukan larutan amelioran penetral.

* **Kriteria Pemicu Karhutla (salah satu terpenuhi):**
  - FWI Level ≥ 2 (TINGGI / SANGAT TINGGI / DARURAT)
  - Kadar air gambut kritis (Soil Moisture < 30%)
  - Suhu tanah gambut panas (DS18B20 ≥ 40°C, indikasi smoldering)

* **Target Volume Rewetting Gambut (Irigasi):**

  | Level FWI | Target Volume |
  |---|---|
  | 2 (TINGGI) | 3.0 Liter |
  | 3 (SANGAT TINGGI) | 5.0 Liter |
  | 4 (DARURAT / Suhu Tanah ≥ 50°C) | 8.0 Liter |

* **Target Volume Amelioran Penetral Asam (Fertigasi):**
  - Default: 1.5 Liter
  - Jika pH < 4.8 (sangat asam): 2.5 Liter

* **Alur Eksekusi:** Irigasi rewetting → Jeda 3 detik → Fertigasi amelioran (eksekusi sekuensial otomatis melalui 1 sensor waterflow).

### 4.3 Fitur Proteksi Keselamatan (*Safety Guards*)

| Proteksi | Deskripsi |
|---|---|
| **Hardware & Software Interlock** | Relay 1 dan Relay 2 saling mengunci, mustahil aktif bersamaan |
| **No-Flow Safety Cutoff** | Jika relay sudah ON 10 detik tapi sensor flow tidak mendeteksi pulsa air (< 0.03 L), relay langsung dimatikan dan alarm MQTT dikirim (`NO_WATER_FLOW`) |
| **Safety Timeout Limit** | Irigasi maks. 3 menit, Fertigasi maks. 2 menit per sesi |
| **Absorption Cooldown** | Jeda serap tanah: 15 menit (pertanian), 5 menit (karhutla darurat) |

---

## 5. Topik MQTT (PubSubClient)

### 5.1 Topik Publikasi (Publish)

| Topik | Isi | Retained |
|---|---|:---:|
| `sproutai/sensor/node/1` | Data sensor Node 1 (Bedeng A) | — |
| `sproutai/sensor/node/2` | Data sensor Node 2 (Bedeng B) | — |
| `sproutai/sensor/node/3` | Data raw sensor + FWI lengkap Node 3 (Karhutla) | — |
| `sproutai/sensor/ph` | Data pH air dari sensor PH-4502C pada Receiver Hub | — |
| `sproutai/sensor/data` | Data sensor multi-node universal | — |
| `sproutai/sensor/flow` | Debit real-time (L/min), volume sesi, total liter, katup aktif | — |
| `sproutai/actuator/status` | Status irigasi, fertigasi, pompa, lampu, mode, target, volume, karhutla_mode | — |
| `sproutai/actuator/alarm` | Alert `NO_WATER_FLOW` saat pompa kering / pipa tersumbat | — |
| `sproutai/karhutla/fwi` | Status analitik FWI lengkap 6 parameter BMKG | ✓ |
| `sproutai/karhutla/alert` | Alert instan saat kategori ≥ TINGGI atau DARURAT | — |
| `sproutai/pompa/status` | Status pompa: `ON` / `OFF` (kompatibilitas dashboard) | — |
| `sproutai/lampu/status` | Status lampu: `ON` / `OFF` | — |
| `sproutai/mode/status` | Mode kerja: `OTOMATIS` / `MANUAL` | — |
| `sproutai/esp32/status` | Status online/offline gateway (LWT) | ✓ |

### 5.2 Topik Langganan (Subscribe)

| Topik | Fungsi |
|---|---|
| `sproutai/pompa/#` | Perintah kontrol irigasi / pompa air bersih (ON/OFF) |
| `sproutai/irigasi/#` | Perintah kontrol irigasi langsung (ON/OFF) |
| `sproutai/fertigasi/#` | Perintah kontrol fertigasi / dosing nutrisi (ON/OFF) |
| `sproutai/lampu/#` | Perintah kontrol lampu grow light (ON/OFF) |
| `sproutai/mode/#` | Pemilihan mode kerja sistem (`AUTO`/`OTOMATIS` / `MANUAL`) |
| `sproutai/ai/#` | Perintah dari AI engine TeranaWeb (e.g. `pump_10s`) |
| `sproutai/weather/#` | Update cuaca eksternal dari BMKG, termasuk kecepatan angin |

---

## 6. Fitur Offline Data Buffering & Operasi Tanpa Internet

Seluruh logika komputasi dan kontrol aktuator dieksekusi **sepenuhnya di chip ESP32** (*edge computing*), sehingga sistem tetap beroperasi 100% normal meskipun koneksi WiFi / internet / MQTT terputus total:

| Komponen | Tetap Berjalan Saat Offline? | Alasan |
|---|:---:|---|
| Penerimaan data dari Sensor Point 1/2/3 | ✅ | ESP-NOW adalah radio peer-to-peer lokal, tidak butuh WiFi |
| Komunikasi Receiver → ESP32 | ✅ | Kabel fisik Hardware UART (bukan jaringan) |
| Perhitungan FWI BMKG Karhutla | ✅ | Komputasi Van Wagner berjalan di CPU ESP32 |
| Kalkulasi volume irigasi & fertigasi | ✅ | Logika sensor + formula berjalan di CPU ESP32 |
| Eksekusi relay dan pembacaan waterflow | ✅ | GPIO langsung, interrupt lokal |
| Tampilan dashboard di layar TFT | ✅ | SPI langsung ke ESP32 |
| Pengiriman data ke HiveMQ Cloud | ❌ | Membutuhkan koneksi WiFi & internet |

Jika koneksi WiFi atau MQTT terputus, ESP32 Gateway secara otomatis:
1. Mendeteksi putusnya koneksi dan memperbarui indikator status pada layar TFT (`WiFi: DISC`, `MQTT: DISC`).
2. Menyimpan hingga **30 paket data** ke dalam memori buffer sirkular (`dataBuffer[]` / `topicBuffer[]`).
3. Terus menerima data sensor dan mengontrol aktuator secara mandiri tanpa gangguan.
4. Saat koneksi internet pulih, seluruh antrean data otomatis di-*flush* ke broker MQTT tanpa kehilangan data dan tanpa memicu watchdog reset.

---

## 7. Prototipe Alat IoT Terintegrasi (Hardware Prototype)

### a. Tiga Unit Sensor Point (ESP8266 / Wemos D1 Mini)

* **Sensor Point 1 (Bedeng A)** dan **Sensor Point 2 (Bedeng B)** masing-masing mengukur suhu dan kelembapan udara (DHT22) serta kelembapan tanah (soil moisture sensor kapasitif). Data ditransmisikan nirkabel menggunakan protokol ESP-NOW pada saluran 1 (*channel 1*) dengan interval pengiriman 1 detik.
* **Sensor Point 3 (Lahan Gambut Karhutla)** mengukur suhu dan kelembapan udara (DHT22), kelembapan tanah gambut (soil moisture kapasitif), serta **suhu tanah gambut bawah permukaan** (DS18B20 waterproof) untuk mendeteksi potensi smoldering fire. Data dikirim dengan interval 2 detik.

### b. Satu Unit Konsentrator dan Pembaca pH (ESP8266 Receiver Hub)

Bertindak sebagai penerima data ESP-NOW dari seluruh Sensor Point, membaca sensor tingkat keasaman air (sensor pH analog PH-4502C) lewat pin analog A0, serta meneruskan seluruh data terstruktur (format JSON) lewat Hardware Serial UART (TX→RX) ke ESP32 Gateway.

### c. Satu Unit Smart Gateway dan HMI Visual (ESP32 DevKit V1 + LCD TFT 3.5" ILI9488)

Menampilkan metrik sensor multi-node, nilai pH air, indeks bahaya karhutla FWI BMKG, status aktuator irigasi dan fertigasi, debit aliran air (L/min) dan total volume kumulatif, mode manual/otomatis, serta status jaringan secara *live* di layar TFT resolusi 480×320 landscape. Sekaligus menjadi jembatan komunikasi aman (TLS port 8883) ke HiveMQ Cloud dengan fitur *offline circular buffering*. Gateway juga langsung mengendalikan 3 relay (irigasi, fertigasi, lampu grow) dan membaca sensor water flow YF-S201 melalui hardware interrupt.

---

## 8. Prosedur Pengujian Sistem

### 8.1 Kalibrasi Sensor Awal

* **Sensor pH PH-4502C:** Probe dibersihkan dengan akuades, lalu dicelupkan berurutan ke larutan buffer standar pH 7.00 dan pH 4.01 untuk mencatat tegangan keluaran dan menentukan faktor kemiringan (*slope*) pada rumus kalibrasi interpolasi linear dua titik:
  ```
  slope = (7.0 - 4.0) / (V_neutral - V_acid)
  pH = slope × voltage + intercept
  ```
* **Sensor Kelembapan Tanah Kapasitif:** Diuji pada kondisi udara kering (probe di udara) dan tanah jenuh air (probe dicelup air) untuk menentukan nilai ADC `SOIL_DRY` dan `SOIL_WET`, memastikan rentang bacaan 0–100%.
* **Sensor Water Flow YF-S201:** Dikalibrasi dengan mengalirkan volume air terukur (gelas ukur 1 Liter) dan memverifikasi jumlah pulsa yang tercatat sesuai spesifikasi (~450 pulsa/Liter).

### 8.2 Pengaktifan Jaringan dan Komunikasi

Sensor Point, Receiver Hub, dan ESP32 Gateway dinyalakan, lalu serial monitor dan layar TFT diamati untuk memastikan bahwa:
* Seluruh Sensor Point terhubung ke Receiver Hub lewat ESP-NOW pada saluran 1.
* Komunikasi UART antara ESP8266 Receiver dan ESP32 Gateway berjalan tanpa paket rusak.
* ESP32 memperoleh alamat IP lokal serta terhubung aman ke HiveMQ Cloud.
* Layar TFT menampilkan data sensor yang diperbarui secara periodik.

### 8.3 Akurasi Sensor

* **DHT22:** Data suhu dan kelembapan udara diambil sebanyak 30 sampel dengan interval, lalu dibandingkan dengan termohigrometer digital terkalibrasi. Galat dinyatakan dengan *Mean Absolute Error* (MAE) dan simpangan baku.
* **Sensor pH:** Diuji pada beberapa sampel cairan dengan keasaman berbeda (air mineral, larutan nutrisi hidroponik, dan air asam/basa ringan), lalu dibandingkan dengan pH meter benchtop laboratorium untuk menilai presisi dan linearitasnya.
* **DS18B20 (Node 3):** Suhu tanah dibandingkan dengan termometer tanah terkalibrasi pada beberapa kedalaman.
* **Water Flow YF-S201:** Diuji akurasi volume air yang tercatat oleh firmware terhadap volume aktual yang diukur secara manual menggunakan gelas ukur pada berbagai laju aliran.

### 8.4 Keandalan Transmisi ESP-NOW

Rasio paket yang berhasil diterima (*Packet Delivery Ratio*) dari tiap Sensor Point ke Receiver Hub diukur pada jarak 5, 10, 20, dan 30 meter, baik pada kondisi pandang langsung (*line-of-sight*) maupun terhalang (*non-line-of-sight*).

### 8.5 Latensi Data End-to-End

Dihitung selisih waktu sejak data dibaca sensor di lapangan, dikirim lewat ESP-NOW, diteruskan lewat UART ke ESP32 Gateway, sampai dipublikasikan lewat MQTT dan tampil pada Web Dashboard TeranaWeb.

### 8.6 Ketahanan Saat Internet Terputus (*Local Operation*)

Koneksi WiFi dimatikan secara sengaja saat sistem berjalan. Diverifikasi bahwa:
* ESP32 mendeteksi putusnya koneksi dan memperbarui status pada layar TFT (`WiFi: DISC`, `MQTT: DISC`).
* Data yang masuk tetap disimpan dalam antrean buffer sirkular (kapasitas 30 paket).
* Seluruh logika aktuator (irigasi, fertigasi, kontrol karhutla) tetap berfungsi tanpa terganggu.
* Setelah WiFi dinyalakan kembali, seluruh data yang tertampung terkirim ke cloud tanpa ada yang hilang dan tanpa memicu watchdog reset.

### 8.7 Respons Aktuator dan Mode Kendali

* **Mode Manual:** Diuji dengan menekan tombol Pompa ON/OFF, Fertigasi ON/OFF, dan Lampu ON/OFF pada Web Dashboard TeranaWeb, lalu memastikan status relay berubah pada layar TFT dan debit air terukur oleh sensor waterflow.
* **Mode Otomatis — Pertanian:** Diuji dengan menyimulasikan tanah kering di bawah threshold (Soil Moisture < 45%) agar siklus irigasi terpicu sendiri, dan menyimulasikan pH di luar rentang optimal (< 5.8 atau > 7.3) agar fertigasi nutrisi terpicu.
* **Mode Otomatis — Karhutla Gambut:** Diuji dengan menyimulasikan skenario FWI Level ≥ 2 dan/atau suhu tanah ≥ 40°C agar siklus gabungan rewetting + amelioran penetral terpicu secara sekuensial.
* **Waktu Tanggap:** Diukur sejak perintah dikirim dari dashboard sampai relay menyalakan perangkat dan indikator TFT berubah.
* **Mekanisme Proteksi:** Safety timeout (maks. 3 menit irigasi, 2 menit fertigasi), no-flow cutoff (alarm jika tidak ada air setelah 10 detik), cooldown jeda peresapan (15 menit pertanian, 5 menit karhutla), dan hardware interlock antar relay diverifikasi.

---

## 9. Dashboard Pemantauan dan Kontrol Berbasis Web (TeranaWeb)

Aplikasi pemantauan dan manajemen pertanian pintar berbasis web responsif (React, Vite, TypeScript, Tailwind CSS) yang terhubung langsung ke broker MQTT melalui saluran terenkripsi WebSocket Secure (WSS port 8884). Menyediakan:

* Visualisasi *live* data sensor multi-node (suhu, kelembaban udara, kelembaban tanah, pH air).
* Peta Peringatan Dini Karhutla Gambut dengan indikator FWI BMKG 5 level warna.
* Panel kontrol manual saklar irigasi, fertigasi, dan lampu grow.
* Pemilihan mode operasi (Otomatis / Manual).
* Monitoring debit aliran air (L/min) dan total volume kumulatif dari sensor waterflow.
* Riwayat telemetri dan log perintah kontrol.
* Integrasi data cuaca BMKG (suhu, kelembaban, kecepatan angin, peluang hujan) untuk rekomendasi penyiraman cerdas dan perhitungan ISI pada indeks karhutla.

---

## 10. Panduan Penggunaan dan Operasional Sistem

### a. Panduan Perangkat Keras

1. **Skema Pengkabelan (Wiring Pinout):** Ikuti tabel diagram pengkabelan pada Bagian 2.1 di atas.
2. **Pemasangan Water Flow Sensor:** Pasang YF-S201 pada pipa utama sebelum percabangan ke solenoid valve, pastikan arah panah pada sensor sesuai arah aliran air.
3. **Prosedur Kalibrasi Sensor Fisik:**
   * Sensor pH: Celupkan berurutan ke larutan buffer pH 7.00 dan pH 4.01, catat tegangan, sesuaikan konstanta `phNeutralVoltage` dan `phAcidVoltage` pada firmware Receiver Hub.
   * Sensor tanah: Catat nilai ADC saat probe di udara kering dan saat dicelup air, sesuaikan `SOIL_DRY` dan `SOIL_WET` pada firmware masing-masing Sensor Point.
4. **Prosedur Penyalaan:** Nyalakan Receiver Hub terlebih dahulu, lalu Sensor Point 1/2/3, dan terakhir ESP32 Gateway. Pastikan layar TFT menampilkan dashboard dan data mulai terisi.
5. **Pembacaan Antarmuka HMI pada Layar TFT Gateway:**
   * Card kiri atas: Node 1 (Bedeng A) — Suhu, Kelembaban Udara, Kelembaban Tanah.
   * Card kanan atas: Node 2 (Bedeng B) — Suhu, Kelembaban Udara, Kelembaban Tanah.
   * Card tengah (border merah): Node 3 Karhutla Gambut — Suhu Tanah, Suhu Udara, Soil%, FWI Score, Status Level BMKG.
   * Card kiri bawah: Sensor pH Air — Keasaman dan Kondisi (Normal/Asam/Basa).
   * Card kanan bawah: Kontrol & Flow — Mode, Debit (L/min), Status Relay 1 (Irigasi), Status Relay 2 (Fertigasi), Total Air kumulatif.
   * Footer: Uptime, jumlah paket diterima, jumlah buffer tertunda.

### b. Panduan Perangkat Lunak dan Aplikasi Web

1. **Akses dan Verifikasi Konektivitas:** Buka TeranaWeb pada browser. Pastikan indikator status menunjukkan `ESP32: Online` dan `MQTT: Connected`.
2. **Navigasi Halaman Monitoring:** Dashboard utama menampilkan data sensor real-time multi-node, status karhutla, dan riwayat telemetri.
3. **Pengoperasian Kontrol Aktuator:** Gunakan tombol Pompa ON/OFF untuk kontrol irigasi manual, dan tombol Fertigasi ON/OFF untuk dosing nutrisi manual. Aktuator akan berhenti otomatis setelah target volume tercapai.
4. **Pemilihan Mode:** Pilih mode Otomatis agar ESP32 menjalankan logika penyiraman cerdas berdasarkan sensor, atau mode Manual agar aktuator hanya merespons perintah dari tombol dashboard.
5. **Pemecahan Masalah Umum:**
   * `WiFi: DISC` pada TFT → Periksa SSID/password WiFi dan jangkauan sinyal.
   * `MQTT: DISC` pada TFT → Periksa koneksi internet, pastikan server HiveMQ Cloud aktif.
   * Data sensor menunjukkan `--.-` → Node belum aktif atau di luar jangkauan ESP-NOW (maks. ~30m LOS).
   * Alarm `NO_WATER_FLOW` → Periksa pompa, solenoid valve, atau kincir sensor flow (apakah ada sumbatan).

---

## 11. Dependensi PlatformIO (`platformio.ini`)

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino

lib_deps =
  knolleary/PubSubClient@^2.8
  bblanchon/ArduinoJson@^7.4.3
  bodmer/TFT_eSPI@^2.5.43
```

---

## 12. Cara Penggunaan & Upload

1. Buka folder ini di **VS Code / PlatformIO**.
2. Salin `include/secret.example.h` menjadi `include/secret.h`, lalu isi kredensial WiFi, MQTT, dan OpenRouter. File `secret.h` diabaikan Git dan tidak boleh dikomit.
3. Hubungkan board ESP32 ke PC melalui kabel data Micro USB.
4. Build dan upload kode:
   ```bash
   pio run --target upload
   ```
5. Buka Serial Monitor (baud rate **115200**) untuk memastikan koneksi Serial2, WiFi, MQTT, inisialisasi aktuator, dan waterflow berjalan lancar:
   ```bash
   pio device monitor -b 115200
   ```
   Output yang diharapkan:
   ```
   =========================================
    NexaGrow ESP32 Gateway with TFT ILI9488
   =========================================
   [UART] Hardware Serial2 initialized on RX=16, TX=17
   [HARDWARE] Aktuator & Waterflow diinisialisasi:
              - Waterflow Pulse Pin : GPIO 27
              - Relay 1 (Irigasi)   : GPIO 25
              - Relay 2 (Fertigasi) : GPIO 26
              - Relay 3 (Lampu)     : GPIO 32
   [WiFi] Connected! IP: 192.168.x.x
   [MQTT] Connecting to HiveMQ... connected!
   ```
6. Untuk menguji keputusan irigasi dan waterflow secara berurutan tanpa mengaktifkan relay fisik, ketik perintah di Serial Monitor dan kirim dengan newline:
   ```text
   SIM start=auto soil=30 critical=35 upper=75 temp=28 rh=65 rain=0 cooldown=ready
   SIM flow_pulses=450 flow_seconds=10
   SIM flow_pulses=1125 flow_seconds=25
   ```
   Perintah pertama mengevaluasi sensor dan memulai simulasi otomatis jika syarat irigasi terpenuhi; perhatikan target liter yang dicetak. Setiap perintah `flow_pulses` adalah **pulsa tambahan sejak input flow sebelumnya**, bukan total kumulatif. Dengan kalibrasi 450 pulsa/liter, dua perintah flow di atas menambahkan total 3,5 L; sesuaikan dengan target yang tampil.
   Parameter keputusan opsional: `soil2`, `critical`, `upper`, `temp`, `rh`, `rain`, `score_on`/`score_off`, `temp_high`/`hum_low`, `cycle=on|off`, dan `cooldown=ready|wait`. Threshold yang tidak diberikan memakai konfigurasi aktif. Cek sesi dengan `SIM status`, atau batalkan/reset dengan `SIM reset`.
   Untuk uji relay fertigasi dan penutupan berdasarkan target dosis:
   ```text
   SIM start=fertigasi target=1.0
   SIM flow_pulses=225 flow_seconds=10
   SIM flow_pulses=225 flow_seconds=10
   ```
   Untuk menguji siklus 3 tahap (pre-wet, dosis pupuk, lalu bilas), gunakan:
   ```text
   SIM start=fertigasi target=1.0 prewet=0.5 flush=0.3
   SIM flow_pulses=225 flow_seconds=10
   SIM flow_pulses=450 flow_seconds=10
   SIM flow_pulses=135 flow_seconds=10
   ```
   Pada fertigasi, `target` wajib dalam liter; `prewet` dan `flush` opsional. Jika `flush` tidak diberikan, relay fertigasi simulasi OFF setelah target dosis tercapai. Mode `start=fertigasi` menguji FSM dosis secara langsung dan **tidak** menguji syarat kelayakan otomatis fertigasi (jadwal biologis, interval hari, pH, cuaca). `start=auto` menguji keputusan irigasi berbasis tanah/cuaca.
   `SIM flow_pulses=0 flow_seconds=36` menguji `NO_FLOW_DETECTED` bila suatu tahap sedang aktif. Semua tahapan dan status relay hanya simulasi; tidak menulis ke pin relay, mengubah data sensor/konfigurasi, atau mengirim MQTT.
7. Untuk uji integrasi nyata dari Serial Monitor (data masuk MQTT/web dan kontrol memakai FSM/relay fisik), sistem harus tersambung MQTT dan disetel **OTOMATIS** dari dashboard. Perintah ini dapat benar-benar mengaktifkan pompa/katup: lakukan hanya di bangku uji aman, dengan area basah dan jalur air dikendalikan serta pengawasan langsung.
   ```text
   SIM live=on confirm=ARM
   SIM node=1 soil=30 temp=28 rh=65
   SIM node=2 soil=34 temp=29 rh=60
   SIM ph=6.5
   SIM rain=0
   SIM run
   SIM flow_pulses=450 flow_seconds=10
   SIM status
   ```
   `SIM node=1|2` menerima `soil`, `temp`, `rh`, `battery`; `SIM node=3` menerima `soil_temp`, `ambient_temp`, `ambient_hum`, `battery`. Nilai node dan pH dipublikasikan ke MQTT/web dengan penanda `simulation=true` dan tidak disimpan ke NVS. `SIM run` meminta satu evaluasi otomatis nyata setelah data dimasukkan; setelahnya FSM berjalan normal. Kirim `flow_pulses` sebagai tambahan pulsa uji (450 pulsa = 1 L); selama live test, pulsa sensor fisik diabaikan agar tidak tercampur dengan input serial. pH yang diinput dipakai oleh logika fertigasi.
   Arm hanya berlaku 10 menit, memerlukan mode OTOMATIS serta koneksi MQTT, dan menahan evaluasi jadwal/otomatis berkala sampai `SIM run` dikirim. FSM aktif, timeout keselamatan, dan waterflow tetap berjalan. `SIM live=off` atau `SIM reset` mematikan aktuator aktif dan melepas arm; jika koneksi MQTT putus atau waktu arm habis, sistem juga disarm dan mematikan aktuator. Data sensor simulasi ditandai stale saat disarm. Untuk benar-benar menguji syarat fertigasi otomatis, waktu NTP, interval hari, pH, cuaca, dan semua interlock tetap harus memenuhi logika firmware.

---

## 13. Penggunaan Memori (Build Terakhir)

```
RAM:   [=         ]  14.6% (used 47960 bytes from 327680 bytes)
Flash: [=======   ]  74.3% (used 973313 bytes from 1310720 bytes)
========================= [SUCCESS] =========================
```
