# TUFAN İKA

İnsansız Kara Aracı (İKA) proje kodları.

## Klasör yapısı

- `arayuz/` — Yer kontrol istasyonu arayüzü (PyQt5, arayüz Jetson'unda çalışır).
  SSH terminal entegrasyonu, telemetri, LiDAR/costmap görselleştirme, uydu
  haritası (Esri World Imagery) içerir.
- `arac_jetson/tufan_v2_ws/` — Araç Jetson'undaki ROS2 (Humble) workspace
  kaynak kodu (Nav2 tabanlı otonom sürüş, IMU/LiDAR/GNSS entegrasyonu,
  motor kontrolü). Build/install/log klasörleri hariç tutuldu (yeniden
  üretilebilir derleme çıktıları).
