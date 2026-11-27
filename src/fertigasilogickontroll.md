## Kondisi A: Node 3 Nonaktif (Hortikultura / Pertanian Biasa / Tanah Mineral)

### Wajib Nyala / Dieksekusi
1. **Jendela Biologis Transpirasi:** Waktu berada pada rentang pagi (`06:30 – 08:30 WIB`) atau sore (`16:00 – 17:30 WIB`).
2. **Interval Nutrisi:** Jadwal hari pemberian nutrisi tercapai (misalnya jadwal pemupukan 2 hari sekali).
3. **Kebutuhan Air:** Kelembapan tanah membutuhkan pasokan air ($\text{skor kebutuhan air} \ge \text{ambang aktif}$).
4. **Rentang pH Aman:** Nilai pH berada dalam batas normal (`5.5 – 7.5`).

### Wajib Mati / Diblokir Total
1. **Peluang Hujan Tinggi:** Prakiraan peluang hujan BMKG $\ge 60\%$ *(mencegah pupuk tercuci / leaching ke air tanah)*.
2. **Tanah Jenuh / Basah:** Kelembapan tanah $\ge 75\%$ *(ambang batas atas)*.
3. **Siang Hari Terik:** Waktu berada pada rentang `11:00 – 14:00 WIB` *(mencegah penguapan ekstrem yang dapat membakar daun atau akar)*.
4. **pH Larutan Ekstrem / Berbahaya:** Nilai $\text{pH} < 4.0$ atau $\text{pH} > 8.5$ *(indikasi alarm tandon bermasalah)*.
5. **No-Flow / Pipa Kosong:** Pompa menyala selama `25 – 30 detik`, tetapi aliran air $< 0.03\text{ Liter}$ *(indikasi tandon habis atau pompa macet)*.
6. **Target / Batas Waktu Tercapai:** Volume target penyiraman sudah terpenuhi atau *safety timeout* (`3 menit`) terlampaui.

---

## Kondisi B: Node 3 Aktif (Lahan Gambut / Mitigasi Karhutla)

### Wajib Nyala / Dieksekusi
1. **Kondisi Darurat Bara Gambut *(Top Priority — Bypass Jadwal Jam & Cuaca)***
   * **Pemicu:** Suhu tanah gambut dalam (sensor DS18B20) $\ge 40^\circ\text{C}$ **ATAU** Indeks Api BMKG ($\text{FWI Level} \ge 2$ / kategori Tinggi–Darurat).
   * **Tindakan:** **Wajib nyala** bersamaan dengan pembasahan (*rewetting* `5 – 8 L` + amelioran `2.5 L`) untuk mendinginkan lapisan dalam gambut dan memadamkan bara api tersembunyi.
2. **Kondisi Pertanian Gambut Masam *(Rutin)***
   * **Pemicu:** Jadwal penyiraman tiba dan sensor membaca $\text{pH} < 4.8$ *(sangat masam khas lahan gambut)*.
   * **Tindakan:** **Wajib nyala** katup fertigasi dengan dosis amelioran dinaikkan sebesar `40 – 50%` untuk menetralkan keasaman.

### Wajib Mati / Diblokir Total
1. **Lahan Tergenang / Banjir:** Kelembapan tanah gambut $\ge 85\%$.
2. **No-Flow Protection:** Aliran macet atau tandon amelioran habis ($\text{flow} < 0.03\text{ L}$ dalam `30 detik`).
3. **Over-dosing Limit:** Volume amelioran per siklus sudah mencapai *hard limit* (maksimal `3.0 Liter`) agar media perakaran tidak kelebihan endapan kapur.