## 4. Success Metrics

Uji Lapangan 1 dinyatakan LULUS bila 8 metrik terpenuhi pada seluruh percobaan.

| ID  | Metrik                            | Cara Menghitung                                                                                                                                                                | Target        | Skenario |
| --- | --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------- | -------- |
| M1  | Jumlah data frame yang masuk      | Kamera dinyalakan selama 5 menit, dan N dihitung melalui:<br>N = (Frame dengan marker terdeteksi / Total frame)                                                                | ≥ 90%         | A        |
| M2  | Settling Time waktu lock on (t_s) | Waktu yang dibutuhkan sejak pertama ArUco berada di kamera hingga gimbal memposisikan ArUco di bagian tengah frame kamera dan berada pada kondisi steady-state (5% error band) | ≤ 2s          | A        |
| M3  | Galat tunak                       | RMS dari error, dengan:<br>e = (jarak titik tengah marker ke titik tengah frame / lebar frame) × 100%                                                                          | ≤ 5%          | A        |
| M4  | Osilasi                           | Selisih puncak-ke-puncak e, dengan:<br>e = (jarak titik tengah marker ke titik tengah frame / lebar frame) × 100%<br>Saat tunak                                                | ≤ 10%         | A        |
| M5  | Pelacakan marker/kamera bergerak  | Error maksimum, dengan:<br>e = (jarak titik tengah marker ke titik tengah frame / lebar frame) × 100%<br>Saat marker digeser ± 15cm dengan kecepatan maksimal 15cm/s           | ≤ 20%         | B, C, D  |
| M6  | Marker tidak lepas                | Jumlah kejadian marker hilang dari frame lebih dari 0.5 detik                                                                                                                  | 0 kejadian    | B, C, D  |
| M7  | Waktu pulih dari gangguan         | Waktu dihitung dari gangguan (geser = ± 30cm) sampai e ≤ 5%                                                                                                                    | ≤ 1,5s        | B, C     |
| M8  | Rentang sudut aman                | Sudut roll, pitch, yaw di log tetap dalam rentang spesifikasi (pitch 0 s/d −100°, roll ±30°, yaw ±90°)                                                                         | 0 Pelanggaran | B, C, D  |