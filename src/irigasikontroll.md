## 1. Perbedaan Saat Node 3 Aktif vs Nonaktif

Sistem menggunakan prinsip prioritas bertingkat:

* **Saat Node 3 AKTIF (Probe Gambut & BMKG Karhutla Terhubung):**
  * Node 3 memegang **Prioritas Tertinggi (Prioritas 1)**.
  * Jika probe mendeteksi tanah gambut mulai panas (>= 40°C) atau indeks kebakaran BMKG bahaya (Level >= 2), ESP32 mendahulukan pembasahan gambut (*rewetting*) sebanyak **3–8 Liter** untuk mencegah kebakaran lahan.
  * Begitu kondisi gambut dinyatakan aman, ESP32 otomatis beralih mengurus lahan hortikultura (Node 1 & 2).

* **Saat Node 3 NONAKTIF (Offline / Rusak / Belum Dipasang):**
  * Sistem tidak macet (*no freeze*). ESP32 langsung melewati (*skip*) bagian gambut dan 100% fokus mengontrol irigasi hortikultura (Node 1 & 2) berdasarkan skor, VPD, fase tanaman, dan jadwal.

---

## 2. Kondisi WAJIB NYALA (Pompa / Katup Hidup)

Pompa/katup irigasi akan menyala jika salah satu kondisi berikut terpenuhi:

* **Darurat Bara Gambut (Prioritas 1 - Khusus Node 3):**
  * Suhu gambut >= 40°C atau indeks bahaya FWI >= 2 -> **Wajib Nyala** untuk pembasahan lahan gambut.
* **Darurat Kekeringan Kritis (Prioritas 2 - Hortikultura / Bypass Jadwal):**
  * Kelembapan tanah pada salah satu titik jatuh di bawah ambang kritis (<= 35%).
  * Tanaman terancam mati layu jika menunggu jam jadwal, sehingga sistem **Wajib Nyala Seketika** untuk penyiraman darurat terukur (**1.2–2.5 Liter**).
* **Jadwal Harian Terpenuhi & Tanaman Butuh Air (Prioritas 4):**
  * Jam dinding (WIB) tepat menyentuh jadwal (misal pukul 06:00) **DAN** skor kebutuhan air tanaman sedang tinggi (>= 25–55 poin) -> **Wajib Nyala** menyiram sesuai target liter.
* **Perintah Manual / AI dari Web Dashboard:**
  * Petani menekan tombol **"ON"** di web dashboard -> **Wajib Nyala**.

---

## 3. Kondisi WAJIB MATI (Pompa / Katup Berhenti / Dibatalkan)

Pompa/katup dilarang menyala atau langsung dimatikan paksa pada kondisi:

* **Target Volume Air Tercapai (*Waterflow Sensor Stop*):**
  * Begitu sensor *waterflow* mendeteksi volume air yang lewat sudah sesuai target liter yang dihitung (misal tepat **1.8 Liter**), katup **Wajib Langsung Mati** agar air tidak terbuang percuma.
* **Proteksi Tanah Sudah Jenuh / Cukup Basah:**
  * Kelembapan tanah sudah >= 75% (ambang atas) -> **Dilarang Nyala / Wajib Batal**, mencegah akar tanaman membusuk dan hemat air.
* **Proteksi Prakiraan Hujan Lebat BMKG:**
  * Peluang hujan >= 85% -> **Dilarang Nyala / Wajib Batal**, karena alam akan segera menyiram tanaman.
* **Proteksi Pompa Kering / Tandon Habis (*No-Flow Protection*):**
  * Pompa menyala selama 35 detik tapi aliran air < 0.03 Liter (tandon kosong / pipa tersumbat) -> **Wajib Mati Darurat Seketika** dan mengirim alarm bahaya ke web, mencegah motor pompa terbakar karena *dry-running*.
* **Proteksi Batas Waktu Darurat (*Safety Timeout*):**
  * Jika kabel sensor *waterflow* terlepas/rusak sehingga tidak menghitung liter, pompa **Wajib Mati Paksa** di menit ke-3 (180 detik) agar lahan tidak kebanjiran.
* **Penyiraman Jadwal Dilewati (Tanah Masih Basah):**
  * Jam jadwal tiba (misal 06:00), tetapi tanah masih lembap sehingga skor kebutuhan air rendah -> **Wajib Tetap Mati / Skip** (menghemat air).


| Input Setting dari Web | Pengaruh Nyata pada Eksekusi Pompa |
| --- | --- |
| `soil_threshold_critical` (Kritis, default: 35%) | Pemicu Siram Darurat: Menentukan garis merah kekeringan. Jika tanah drop ke angka ini, jadwal jam diabaikan dan pompa langsung menyiram darurat. |
| `soil_threshold_high` (Atas, default: 75%) | Penentu Stop & Volume Air: Menjadi batas atas tanah jenuh (stop siram) dan menentukan berapa Liter air yang harus dialirkan sensor waterflow (ΔSM\\Delta SMΔSM). |
| `soil_threshold_low` (Bawah, default: 45%) | Batas Nyaman Tanaman: Menjadi batas toleransi awal sebelum tanaman mulai merasakan kekurangan air. |
| `temp_threshold_high` (Suhu Atas, default: 34°C) | Kompensasi Panas Terik: Jika suhu udara lahan melebihi batas ini, laju penguapan sangat cepat. ESP32 otomatis menambah target air waterflow (+15%) agar air tidak habis menguap. |
| `humidity_threshold_low` (RH Bawah, default: 60%) | Kompensasi Udara Kering: Jika udara terlalu kering, ESP32 menambah target air waterflow (+10%) untuk melawan penguapan transpirasi daun. |
| `plant_phase` (Vegetatif vs Generatif) | Karakter Penyiraman:<br>• Vegetatif: Ambang skor 55 (air melimpah 1.0×1.0\\times1.0× untuk pembentukan akar/daun).<br>• Generatif: Ambang skor 60 (air lebih hemat 0.85×0.85\\times0.85× agar bunga/buah tidak rontok). |
| `watering_time` & `watering_enabled` | Pengatur Jadwal Harian: Menentukan jam rutin penyiraman (misal `06:00 WIB`) dan apakah fitur jadwal aktif atau nonaktif. |