import sys, re, math, time
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QMessageBox, QSizePolicy, QWidget, QFrame, QPushButton
from PyQt5.QtCore import Qt, QRectF, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPainterPath, QFont, QColor, QPen
from datetime import datetime

# --- ROS 2 HUMBLE VE LIDAR KÜTÜPHANELERİ ---
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import LaserScan
except Exception as e:
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    LaserScan = None
    ROS2_IMPORT_ERROR = str(e)
else:
    ROS2_IMPORT_ERROR = None

# Yan odalardaki işçileri çağırıyoruz
from kamera_sistemi import KameraThread
from telemetri_sistemi import TelemetriThread
from harita_sistemi import HaritaYoneticisi
from surus_joystick_sistemi import SurusJoystickThread
from stiller import *
from arayuz import Ui_MainWindow
from diller import CEVIRILER
from terminal_widget import SshTerminalWidget


# ==========================================
# ROS 2 LIDAR DİNLEYİCİ THREAD (ARAYÜZÜ KORUR)
# ==========================================
class LidarThread(QThread):
    scan_sinyali = pyqtSignal(list, list)  # (Açılar, Mesafeler) sinyali

    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.calisiyor = True

    def run(self):
        if rclpy is None or Node is None or QoSProfile is None or QoSReliabilityPolicy is None or LaserScan is None:
            print("[LIDAR] ROS2 kütüphanesi bulunamadığı için lidar dinleyicisi başlatılamadı.")
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('gcs_lidar_subscriber')

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=10
            )

            self.sub = self.node.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                qos_profile
            )
            # KENDİ ÖZEL EXECUTOR: harita_sistemi.py'deki Ros2GcsMotoru ile aynı anda
            # rclpy'nin paylaşılan global executor'ına spin edilirse ("rclpy.spin(node)")
            # iki thread aynı wait-set'e dokunup "IndexError: wait set index too big"
            # ile çöküyor. Kendi executor'ımızla bu thread'i izole ediyoruz.
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            while rclpy.ok() and self.calisiyor:
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"Lidar ROS2 Thread Hatası: {e}")

    def stop(self):
        self.calisiyor = False
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        self.quit()
        self.wait()

    def scan_callback(self, msg):
        # Ham açı ve mesafe verilerini alıp GUI'ye sinyal atıyoruz
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        self.scan_sinyali.emit(angles, list(msg.ranges))


# ==========================================
# ANA YER KONTROL İSTASYONU SINIFI
# ==========================================
class TufanGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # --- LOG EKRANINI GERÇEK SSH TERMİNALİNE ÇEVİR ---
        # Araç Jetson'una hiçbir zaman monitör bağlanmayacağı için, eski
        # append-only log kutusu yerine buradan doğrudan komut çalıştırılabilen
        # gerçek bir terminal (pyte + pty + ssh) koyuyoruz. arayuz.py pyuic5
        # ile otomatik üretildiği için oradaki QTextEdit'i burada, setupUi()
        # sonrasında değiştiriyoruz (arayuz.py yeniden üretilse bile bu kod
        # çalışmaya devam eder).
        eski_terminal = self.ui.terminal_ekrani
        self.ui.gridLayout_4.removeWidget(eski_terminal)
        eski_terminal.deleteLater()
        self.ui.terminal_ekrani = SshTerminalWidget(self.ui.log_ekran)
        self.ui.gridLayout_4.addWidget(self.ui.terminal_ekrani, 0, 0, 1, 1)

        # Sayfa her değiştiğinde bu fonksiyonu otomatik çalıştır:
        self.ui.stackedWidget.currentChanged.connect(self.sayfa_odak_ayarla)
        self.showFullScreen()
        
        # SİSTEM YAZISI İÇİN BETON YÖNTEM
        sistem_font = self.ui.label_sistemYazi.font()
        sistem_font.setPixelSize(32)  
        sistem_font.setBold(True)
        self.ui.label_sistemYazi.setFont(sistem_font)

        # ŞARJ YÜZDELERİ İÇİN BETON YÖNTEM
        sarj_font = self.ui.label_aracSarj.font()
        sarj_font.setPixelSize(26)
        sarj_font.setBold(True)
        self.ui.label_aracSarj.setFont(sarj_font)
        self.ui.label_yerSarj.setFont(sarj_font)
       
        # --- BAŞLANGIÇ AYARLARI ---
        self.setWindowTitle("TUFAN Yer Kontrol İstasyonu")
        self.resize(1920, 1080)

        # ARAYÜZÜ ZORLA ANA MENÜ (0. SAYFA) İLE BAŞLAT!
        self.ui.stackedWidget.setCurrentIndex(0)
        self.sayfa_odak_ayarla(0) # Klavyeyi de anında Ana Ekrana kilitle

        # --- HAFIZA BAYRAKLARI ---
        self.kayit_yapiyor_mu = False
        self.yedekleme_yapiyor_mu = False
        self.bildirim_acik = False   
        self.joystick_bagli = False  
        self.klavye_aktif = False    
        self.sifreleme_acik = True  

        # --- LİDAR HAFIZA DEĞİŞKENLERİ ---
        self.lidar_acilar = []
        self.lidar_mesafeler = []
        self.lidar_max_mesafe = 10.0  # Radar kenarı kaç metreye denk gelsin (Örn: 10m)

        # Kameralar türkçe başlasın
        self.kamera_yazi_on = "ÖN"
        self.kamera_yazi_arka = "ARKA"
        self.kamera_yazi_silah = "SİLAH" 

        self.current_lang = "Türkçe"
        self.aktif_dil = "Türkçe"
        
        self._baslangic_stilleri_uygula()
        self.arayuz_esneklik_ayarlarini_uygula()

        self._telemetri_baglantilari_kur()
        self._buton_baglantilarini_kur()

        self._radar_zamanlayici_baslat()
        self._kamera_ve_joystick_baslat()
        self._lidar_motoru_baslat() # ROS 2 Lidar motorunu başlat
        
        # --- DONANIM SİSTEMLERİNİ BAŞLAT ---
        self.harita_kurulumu()
        self._kamera_tuvallerini_kur()

        # Joystick butonuna, sağlam olan Klavye butonunun tüm genetiğini zorla kopyala!
        self.ui.pushButton_joystick.setFont(self.ui.pushButton_klavye.font())
        self.ui.pushButton_joystick.setSizePolicy(self.ui.pushButton_klavye.sizePolicy())

        # BAŞLANGIÇ MESAJLARI
        self.log_yaz("TUFAN Yer Kontrol İstasyonu Başlatıldı...")
        self.log_yaz("Sistem Durumu: HAZIR")

        self._tasarim_hafizasini_kaydet()

    # ==========================================
    # FONKSİYONLAR
    # ==========================================

    def log_yaz(self, mesaj):
        su_an = datetime.now().strftime("%H:%M:%S")
        self.ui.terminal_ekrani.append(f"[{su_an}] {mesaj}")

    def akilli_bildirim_gonder(self, baslik, mesaj, kritik_mi=False):
        self.log_yaz(f"{baslik}: {mesaj}")
        if self.bildirim_acik:
            uyari_kutusu = QMessageBox(self)
            uyari_kutusu.setWindowTitle(baslik)
            uyari_kutusu.setText(mesaj)
            uyari_kutusu.setStyleSheet("""
                QMessageBox { background-color: #1e1e1e; }
                QLabel { color: white; font-weight: bold; font-size: 16px; }
                QPushButton { background-color: #00a8ff; color: white; border-radius: 5px; padding: 5px 15px; font-weight: bold; }
                QPushButton:hover { background-color: #008ecc; }
            """)
            if kritik_mi:
                uyari_kutusu.setIcon(QMessageBox.Critical)
            else:
                uyari_kutusu.setIcon(QMessageBox.Information)
            uyari_kutusu.show()

    def arayuz_esneklik_ayarlarini_uygula(self):
        basliklar = [
            self.ui.label_ayarlarYazi, self.ui.label_genelAyarlarYazi, 
            self.ui.label_kontrolAyarlariYazi, self.ui.label_guvenlikKontrolleriYazi, 
            self.ui.label_goruntuAyarlari, self.ui.label_veriYedeklemeYazi,
            self.ui.label_arac, self.ui.label_yer
        ]
        for baslik in basliklar:
            baslik.setAlignment(Qt.AlignCenter)
            baslik.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        
        butonlar = [
            self.ui.pushButton_bildirimler, self.ui.pushButton_sifreleme,
            self.ui.pushButton_klavye, self.ui.pushButton_joystick,
            self.ui.pushButton_KAYIT, self.ui.pushButton_simdiYedekle
        ]
        for buton in butonlar:
            buton.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def _baslangic_stilleri_uygula(self):       
        self.ui.pushButton_klavye.setStyleSheet(AYAR_PASIF)
        self.ui.pushButton_klavye.setText("Pasif")
        self.ui.pushButton_joystick.setStyleSheet(AYAR_PASIF)
        self.ui.pushButton_joystick.setText("Bağlantı Yok")
        self.ui.pushButton_bildirimler.setStyleSheet(AYAR_PASIF)
        self.ui.pushButton_bildirimler.setText("Kapalı")
        self.ui.frame_anaKamera.setStyleSheet("background-color: #1e1e1e; border: 2px solid red; border-radius: 32px;")
        self.ui.frame_yanKamera1.setStyleSheet("background-color: #0F4C81; border-radius: 32px;") 
        self.ui.frame_yanKamera2.setStyleSheet("background-color: #008cba; border-radius: 32px;") 
        self.ui.frame_yanKamera3.setStyleSheet("background-color: #2C3E50; border-radius: 32px;") 

    def _telemetri_baglantilari_kur(self):  
        self.telemetri_motoru = TelemetriThread()
        self.telemetri_motoru.hiz_sinyali.connect(self.ui.label_hizYazi.setText)
        self.telemetri_motoru.batarya_sinyali.connect(self.arac_batarya_renklendir)
        self.telemetri_motoru.etap_sinyali.connect(self.ui.label_etapNo.setText) 
        self.telemetri_motoru.yer_batarya_sinyali.connect(self.yer_batarya_renklendir)
        self.telemetri_motoru.imu_durum_sinyali.connect(self.imu_arayuz_guncelle)
        self.telemetri_motoru.guc_durum_sinyali.connect(self.guc_arayuz_guncelle)
        self.telemetri_motoru.gps_durum_sinyali.connect(self.gps_arayuz_guncelle)
        self.telemetri_motoru.kamera_durum_sinyali.connect(self.kamera_arayuz_guncelle)
        self.telemetri_motoru.wifi_durum_sinyali.connect(self.wifi_arayuz_guncelle)
        self.telemetri_motoru.lidar_durum_sinyali.connect(self.lidar_arayuz_guncelle)
        self.telemetri_motoru.mod_durum_sinyali.connect(self.mod_durum_guncelle)
        self.telemetri_motoru.hedef_mesafe_sinyali.connect(self.hedef_mesafe_guncelle)
        self.son_hedef_mesafe = None
        self.son_hedef_mesafe_zamani = 0.0
        # Gercek heartbeat'ler ilk kez tetiklenmeden once panel tasarim-zamani
        # varsayilan metinlerini gostermeye devam etmesin diye baslangicta
        # acikca PASIF'e cekiyoruz.
        self.lidar_arayuz_guncelle(False)
        self.imu_arayuz_guncelle(False)
        self.mod_durum_guncelle(False, False)
        
        # --- GÜNCELLEME 1: TELEMETRİ LOGLARINI TERMINALE BAĞLA ---
        # Tuşlara basıldığında palet hız bildirimlerinin GUI terminaline düşmesi sağlandı!
        self.telemetri_motoru.log_sinyali.connect(self.log_yaz)
        
        self.telemetri_motoru.start()

    def _buton_baglantilarini_kur(self):
        self.ui.home_button.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.kamera_button.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.navigasyon_button.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(2))
        self.ui.hakkinda_button.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.ayarlar_button.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(4))

        self.ui.pushButton_bildirimler.clicked.connect(self.ayar_bildirim_tetikle)
        self.ui.pushButton_joystick.clicked.connect(self.ayar_joystick_tetikle)
        self.ui.pushButton_klavye.clicked.connect(self.ayar_klavye_tetikle)
        self.ui.pushButton_sifreleme.clicked.connect(self.ayar_sifreleme_tetikle)

        self.ui.pushButton_KAYIT.clicked.connect(self.veri_kaydet_tetikle)
        self.ui.pushButton_simdiYedekle.clicked.connect(self.yedekleme_tetikle)

        self.ui.pushButton_manuel.clicked.connect(self.manuel_sec)
        self.ui.pushButton_Otonom.clicked.connect(self.otonom_sec)
        self.ui.power_button.clicked.connect(self.istasyonu_kapat)
        self.ui.pushButton_acilKapat.clicked.connect(self.acil_durdurma)

        self.ui.comboBox_temaSecimi.currentIndexChanged.connect(self.tema_degistir)
        self.ui.comboBox_dilSecimi.currentTextChanged.connect(self.dil_degistir)

        # Eskiden "Telefon" kutusu - kullanilmiyordu, kolay surus icin
        # PWM ust sinir girisine cevrildi (bkz. pwm_sinirini_uygula).
        # NOT: label_telefon.setText() normalde SADECE dil_degistir()
        # icinde cagriliyor (dil degistirilene kadar arayuz.py'nin sabit
        # "Telefon:" yazisi kalir) - bu yuzden ilk acilista da burada
        # ACIKCA guncelliyoruz.
        dil_baslangic = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        self.ui.label_telefon.setText(dil_baslangic["telefon"])
        self.ui.label_telefon.setStyleSheet(
            "color: #FFB454; font-size: 24px; font-weight: bold; "
            "border: none; background-color: transparent;"
        )
        self.ui.lineEdit_kullaniciAdi_2.setText(f"{self.telemetri_motoru.pwm_ust_sinir:.0f}")
        self.ui.lineEdit_kullaniciAdi_2.setStyleSheet(PWM_LIMIT_VARSAYILAN)
        self.ui.lineEdit_kullaniciAdi_2.editingFinished.connect(self.pwm_sinirini_uygula)

    def _kamera_tuvallerini_kur(self):
        self.aktif_kamera = 1  

        self.kamera_layout_ana = QVBoxLayout(self.ui.frame_anaKamera)
        self.kamera_layout_ana.setContentsMargins(0, 0, 0, 0)
        self.tuval_ana = QLabel()
        self.tuval_ana.setAlignment(Qt.AlignCenter)
        self.tuval_ana.setStyleSheet("color: white; font-size: 20px; font-weight: bold;")
        self.tuval_ana.setScaledContents(True)
        self.kamera_layout_ana.addWidget(self.tuval_ana)

        self.kamera_layout_sag1 = QVBoxLayout(self.ui.frame_yanKamera1)
        self.kamera_layout_sag1.setContentsMargins(0, 0, 0, 0)
        self.tuval_sag1 = QLabel()
        self.tuval_sag1.setAlignment(Qt.AlignCenter)
        self.tuval_sag1.setStyleSheet("color: white; font-weight: bold;")
        self.tuval_sag1.setScaledContents(True)
        self.kamera_layout_sag1.addWidget(self.tuval_sag1)

        self.kamera_layout_sag2 = QVBoxLayout(self.ui.frame_yanKamera2)
        self.kamera_layout_sag2.setContentsMargins(0, 0, 0, 0)
        self.tuval_sag2 = QLabel()
        self.tuval_sag2.setAlignment(Qt.AlignCenter)
        self.tuval_sag2.setStyleSheet("color: #00bfff; font-weight: bold;")
        self.tuval_sag2.setScaledContents(True)
        self.kamera_layout_sag2.addWidget(self.tuval_sag2)

        self.kamera_layout_sag3 = QVBoxLayout(self.ui.frame_yanKamera3)
        self.kamera_layout_sag3.setContentsMargins(0, 0, 0, 0)
        self.tuval_sag3 = QLabel()
        self.tuval_sag3.setAlignment(Qt.AlignCenter)
        self.tuval_sag3.setStyleSheet("color: #00bfff; font-weight: bold;")
        self.tuval_sag3.setScaledContents(True)
        self.kamera_layout_sag3.addWidget(self.tuval_sag3)

        self.ui.frame_yanKamera1.mousePressEvent = lambda event: self.kamera_sec(1)
        self.ui.frame_yanKamera2.mousePressEvent = lambda event: self.kamera_sec(2)
        self.ui.frame_yanKamera3.mousePressEvent = lambda event: self.kamera_sec(3)

        self.kamera_sec(1)

    def _radar_zamanlayici_baslat(self):
        self.radar_aci = 0
        self.radar_zamanlayici = QTimer()
        self.radar_zamanlayici.timeout.connect(self.radar_cizimi_Guncelle)
        self.radar_zamanlayici.start(50)

    def _kamera_ve_joystick_baslat(self):
        self.kamera_motoru = KameraThread(udp_ip="0.0.0.0", udp_port=5000, tabela_ip="0.0.0.0", tabela_port=5001)
        self.kamera_motoru.kare_sinyali.connect(self.video_ekrana_bas)
        self.kamera_motoru.start()

        # KAMERA durumu: UDP'den (silah/ana kamera) gercekten kare gelip
        # gelmedigine (heartbeat) gore -- veri kesilirse 1.5sn icinde BAĞLANTI
        # KOPTU'ya doner.
        self.son_kamera_frame_zamani = 0.0
        self.kamera_heartbeat_zamanlayici = QTimer()
        self.kamera_heartbeat_zamanlayici.timeout.connect(self._kamera_heartbeat_kontrol)
        self.kamera_heartbeat_zamanlayici.start(500)

    def _kamera_heartbeat_kontrol(self):
        hazir_mi = (time.time() - self.son_kamera_frame_zamani) < 1.5
        self.kamera_arayuz_guncelle(hazir_mi)

    # --- ROS 2 LİDAR THREAD BAŞLATICI ---
    def _lidar_motoru_baslat(self):
        try:
            self.lidar_motoru = LidarThread()
            self.lidar_motoru.scan_sinyali.connect(self.lidar_verisi_guncelle)
            self.lidar_motoru.start()
            self.log_yaz("Sistem: ROS 2 Lidar dinleyicisi (/scan - Reliable) başlatıldı.")
        except Exception as e:
            self.log_yaz(f"HATA: Lidar motoru başlatılamadı: {e}")

    def lidar_verisi_guncelle(self, acilar, mesafeler):
        self.lidar_acilar = acilar
        self.lidar_mesafeler = mesafeler
        # NOT: burada lidar_arayuz_guncelle(True) ARTIK ÇAĞRILMIYOR -- bu çağrı
        # veri geldiğinde hep True yapıyordu ama veri KESİLİNCE hiçbir zaman
        # False'a dönmüyordu (sahte "her zaman aktif" görüntüsü veriyordu).
        # LİDAR durumu artık TEK kaynaktan: telemetri_motoru.lidar_durum_sinyali
        # (gerçek /scan heartbeat, hem aktif HEM pasif durumu doğru yakalıyor).

    def _tasarim_hafizasini_kaydet(self):
        self.orijinal_tasarim_hafizasi = {}
        for widget in self.findChildren(QWidget):
            if widget.objectName():
                self.orijinal_tasarim_hafizasi[widget.objectName()] = widget.styleSheet()

    # ==========================================
    # ANA EKRAN TELEMETRİ GÜNCELLEME FONKSİYONLARI
    # ==========================================
    def imu_arayuz_guncelle(self, aktif_mi):
        # Eskiden "MOTORLAR" olan bu gosterge artik gercek /imu/data
        # heartbeat'ine gore IMU durumunu gosteriyor.
        dil = getattr(self, 'current_lang', 'Türkçe')
        durum = "ACTIVE" if aktif_mi else ("PASSIVE" if dil == "English" else "PASİF")
        self.ui.label_motorYazi.setText(f"IMU : {durum}")
        self.ui.label_motorYazi.setStyleSheet(f"color: {'#00ff00' if aktif_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def guc_arayuz_guncelle(self, normal_mi):
        dil = getattr(self, 'current_lang', 'Türkçe')
        baslik = "POWER SYSTEM" if dil == "English" else "GÜÇ SİSTEMİ"
        durum = "NORMAL" if normal_mi else ("CRITICAL" if dil == "English" else "KRİTİK")
        self.ui.label_GucYazi.setText(f"{baslik} : {durum}")
        self.ui.label_GucYazi.setStyleSheet(f"color: {'#00ff00' if normal_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def gps_arayuz_guncelle(self, etkin_mi):
        dil = getattr(self, 'current_lang', 'Türkçe')
        durum = "ACTIVE" if etkin_mi else ("PASSIVE" if dil == "English" else "PASİF")
        self.ui.label_gpsYazi.setText(f"GPS : {durum}")
        self.ui.label_gpsYazi.setStyleSheet(f"color: {'#00ff00' if etkin_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")
    
    def wifi_arayuz_guncelle(self, yuzde):
        self.ui.label_wifiYazi.setText(f"WIFI : %{yuzde}")
        if yuzde > 60:
            self.ui.label_wifiYazi.setStyleSheet("color: #00ff00; font-weight: bold; font-size: 24px; border: none; background-color: transparent;") 
        elif yuzde > 30:
            self.ui.label_wifiYazi.setStyleSheet("color: orange; font-weight: bold; font-size: 24px; border: none; background-color: transparent;") 
        else:
            self.ui.label_wifiYazi.setStyleSheet("color: red; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def kamera_arayuz_guncelle(self, hazir_mi):
        dil = getattr(self, 'current_lang', 'Türkçe')
        baslik = "CAMERA" if dil == "English" else "KAMERA"
        durum = "READY" if hazir_mi else ("ERROR" if dil == "English" else "BAĞLANTI KOPTU")
        self.ui.label_kameraYazi.setText(f"{baslik} : {durum}")
        self.ui.label_kameraYazi.setStyleSheet(f"color: {'#00ff00' if hazir_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def lidar_arayuz_guncelle(self, aktif_mi):
        dil = getattr(self, 'current_lang', 'Türkçe')
        baslik = "LIDAR" if dil == "English" else "LİDAR"
        if aktif_mi:
            durum = "SCANNING..." if dil == "English" else "TARANIYOR..."
            renk = "#00AAFF"
        else:
            durum = "PASSIVE" if dil == "English" else "PASİF"
            renk = "red"
        self.ui.label_lidarYazi.setText(f"{baslik} : {durum}")
        self.ui.label_lidarYazi.setStyleSheet(f"color: {renk}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def hedef_mesafe_guncelle(self, mesafe):
        self.son_hedef_mesafe = mesafe
        self.son_hedef_mesafe_zamani = time.time()

    def mod_durum_guncelle(self, otonom_hazir, manuel_hazir):
        # Eskiden genel PASİF/AKTİF heartbeat'i gosteren "SİSTEM" gostergesi,
        # artik aractaki Jetson'da hangi surus modunun (otonom: goal_manager
        # heartbeat'i / manuel: arduino seri baglantisi) gercekten calisir
        # durumda oldugunu gosteriyor. Ikisi de hazirsa aralarinda donusumlu
        # yazi gosterir (bkz. _mod_cycle_zamanlayici).
        self._otonom_hazir = otonom_hazir
        self._manuel_hazir = manuel_hazir
        if not hasattr(self, 'mod_cycle_zamanlayici'):
            self.mod_cycle_zamanlayici = QTimer()
            self.mod_cycle_zamanlayici.timeout.connect(self._mod_cycle_tetikle)
            self._mod_cycle_goster_otonom = True

        if otonom_hazir and manuel_hazir:
            if not self.mod_cycle_zamanlayici.isActive():
                self._mod_cycle_goster_otonom = True
                self.mod_cycle_zamanlayici.start(2000)
        else:
            self.mod_cycle_zamanlayici.stop()
        self._mod_metni_ciz()

    def _mod_cycle_tetikle(self):
        self._mod_cycle_goster_otonom = not self._mod_cycle_goster_otonom
        self._mod_metni_ciz()

    def _mod_metni_ciz(self):
        dil = getattr(self, 'current_lang', 'Türkçe')
        baslik = "SYSTEM" if dil == "English" else "SİSTEM"
        otonom_metni = "AUTONOMOUS READY" if dil == "English" else "OTONOM HAZIR"
        manuel_metni = "MANUAL READY" if dil == "English" else "MANUEL HAZIR"

        if self._otonom_hazir and self._manuel_hazir:
            if self._mod_cycle_goster_otonom:
                durum, renk = otonom_metni, "#00AAFF"
            else:
                durum, renk = manuel_metni, "#00ff00"
        elif self._otonom_hazir:
            durum, renk = otonom_metni, "#00AAFF"
        elif self._manuel_hazir:
            durum, renk = manuel_metni, "#00ff00"
        else:
            durum, renk = ("PASSIVE" if dil == "English" else "PASİF"), "red"

        self.ui.label_sistemYazi.setText(f"{baslik} : {durum}")
        self.ui.label_sistemYazi.setStyleSheet(f"color: {renk}; font-weight: bold;")

    def manuel_sec(self):
        self.ui.pushButton_manuel.setStyleSheet(MOD_AKTIF)
        self.ui.pushButton_Otonom.setStyleSheet(MOD_PASIF)
        self.log_yaz("Sürüş Modu: MANUEL KONTROL AKTİF")
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.mod_degistir("MANUEL")
            
        # 1. DÜZELTME: Manuele geçince klavye kilitliyse otomatik olarak GERİ AÇ!
        if not getattr(self, 'klavye_aktif', False):
            self.ayar_klavye_tetikle()
            
        # 2. DÜZELTME: Fareyle butona tıkladığın için kaybolan klavye odağını (focus) anında ana ekrana geri çek!
        self.setFocus()

    def otonom_sec(self):
        self.ui.pushButton_Otonom.setStyleSheet(MOD_AKTIF)
        self.ui.pushButton_manuel.setStyleSheet(MOD_PASIF)
        self.log_yaz("Sürüş Modu: OTONOM SEYRÜSEFER AKTİF")
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.mod_degistir("OTONOM")
        
        # Otonoma geçince güvenlik için klavyeyi otomatik kilitle
        if getattr(self, 'klavye_aktif', False):
            self.ayar_klavye_tetikle()
            
        self.setFocus()

    def istasyonu_kapat(self):
        self.log_yaz("Sistem kapatılıyor...")
        self.close()

    def acil_durdurma(self):
        self.ui.terminal_ekrani.append("<br><span style='color: red; font-size: 16px;'><b>!!! ACİL DURUM: TÜM SİSTEMLER DURDURULDU !!!</b></span>")
        self.ui.terminal_ekrani.append("<span style='color: red;'><b>MOTOR GÜCÜ KESİLDİ - ARAÇ GÜVENLİ MODA GEÇTİ</b></span><br>")
        for _ in range(3):
            QApplication.beep()
        self.akilli_bildirim_gonder(
            "🛑 !!! ACİL DURUM !!!", 
            "ARAÇTAKİ JETSON'A KAPATMA EMRİ GÖNDERİLDİ!\nMOTOR SÜRÜCÜLERİNİN GÜCÜ KESİLDİ.", 
            kritik_mi=True
        )
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.hareket_emri_gonder("EMERGENCY_STOP_CMD")

    def veri_kaydet_tetikle(self):
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        if not self.kayit_yapiyor_mu:
            self.log_yaz("VİDEO KAYDI BAŞLATILDI: Veriler diske yazılıyor...")
            self.ui.pushButton_KAYIT.setStyleSheet(KAYIT_AKTIF)
            self.ui.pushButton_KAYIT.setText(dil["btn_kaydi_durdur"])
            self.kayit_yapiyor_mu = True
            if hasattr(self, 'kamera_motoru'):
                self.kamera_motoru.kayit_durumu_degistir(True)
        else:
            self.log_yaz("VİDEO KAYDI DURDURULDU: Dosya başarıyla kaydedildi.")
            self.ui.pushButton_KAYIT.setStyleSheet(KAYIT_PASIF)
            self.ui.pushButton_KAYIT.setText(dil["btn_kaydet"])
            self.kayit_yapiyor_mu = False
            if hasattr(self, 'kamera_motoru'):
                self.kamera_motoru.kayit_durumu_degistir(False)

    def yedekleme_tetikle(self):
        hedef_klasor = self.ui.lineEdit_hedefKlasor.text() 
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        if not self.yedekleme_yapiyor_mu:
            self.ui.pushButton_simdiYedekle.setStyleSheet(YEDEK_AKTIF)
            self.ui.pushButton_simdiYedekle.setText(dil["btn_yedeklemeyi_durdur"])
            self.log_yaz(f"YEDEKLEME BAŞLADI: Video kaydı {hedef_klasor} dizini için alınıyor...")
            self.yedekleme_yapiyor_mu = True
            if hasattr(self, 'kamera_motoru'):
                self.kamera_motoru.kayit_durumu_degistir(True)
        else:
            self.ui.pushButton_simdiYedekle.setStyleSheet(YEDEK_PASIF)
            self.ui.pushButton_simdiYedekle.setText(dil["btn_yedekle"])
            self.log_yaz("YEDEKLEME DURDURULDU: Video dosyası başarıyla paketlendi.")
            self.yedekleme_yapiyor_mu = False
            if hasattr(self, 'kamera_motoru'):
                self.kamera_motoru.kayit_durumu_degistir(False)

    def harita_kurulumu(self):
        self.log_yaz("Sistem: Operasyon Haritası yükleniyor...")
        self.harita_layout = QVBoxLayout(self.ui.frame_navigasyonAlan)
        self.harita_layout.setContentsMargins(0, 0, 0, 0)
        self.harita_yoneticisi = HaritaYoneticisi(self.harita_layout)
        self.log_yaz("Sistem: Harita başarıyla ana ekrana yüklendi.")

    def arac_batarya_renklendir(self, gelen_metin):
        sayilar = re.findall(r'\d+', str(gelen_metin))
        yuzde = int(sayilar[0]) if sayilar else 0

        if yuzde > 60:
            renk = "#00ff00" 
        elif yuzde > 30:
            renk = "#FFD700" 
        elif yuzde > 15:
            renk = "#FFA500" 
        else:
            renk = "#ff0000" 

        self.ui.label_aracSarj.setStyleSheet(f"color: {renk}; font-weight: bold;")
        self.ui.label_aracSarj.setText(gelen_metin)

    def yer_batarya_renklendir(self, gelen_metin):
        sayilar = re.findall(r'\d+', str(gelen_metin))
        yuzde = int(sayilar[0]) if sayilar else 0

        if yuzde > 60:
            renk = "#00ff00" 
        elif yuzde > 30:
            renk = "#FFD700" 
        elif yuzde > 15:
            renk = "#FFA500" 
        else:
            renk = "#ff0000" 

        self.ui.label_yerSarj.setStyleSheet(f"color: {renk}; font-weight: bold;")
        self.ui.label_yerSarj.setText(gelen_metin)

    def dil_degistir(self, secilen_dil):
        self.aktif_dil = secilen_dil
        self.current_lang = secilen_dil
        dil = CEVIRILER.get(secilen_dil, CEVIRILER["Türkçe"])
        
        self.ui.label_ayarlarYazi.setText(dil["ana_baslik"])
        self.ui.label_sistemDili.setText(dil["sistem_dili"])
        self.ui.label_tema.setText(dil["tema"])
        durum_bildirim = getattr(self, 'bildirim_acik', False)
        self.ui.pushButton_bildirimler.setText(dil["btn_acik"] if durum_bildirim else dil["btn_kapali"])
        stil_bildirim = AYAR_AKTIF if durum_bildirim else AYAR_PASIF
        self.ui.pushButton_bildirimler.setStyleSheet(stil_bildirim + " font-size: 16px; font-weight: bold;")
        
        self.ui.label_kontrolAyarlariYazi.setText(dil["kontrol_ayarlari"])
        self.ui.label_klavyeKontrol.setText(dil["klavye_kontrol"])
        self.ui.pushButton_manuel.setText(dil["mod_manuel"])
        self.ui.pushButton_Otonom.setText(dil["mod_otonom"])

        self.kamera_yazi_on = dil["kamera_on"]
        self.kamera_yazi_arka = dil["kamera_arka"]
        self.kamera_yazi_silah = dil["kamera_silah"]
        self.ui.pushButton_KAYIT.setText(dil["veri_kaydet"])
        self.ui.label_baslik.setText("TUFAN YER KONTROL İSTASYONU" if self.current_lang == "Türkçe" else "TUFAN GROUND CONTROL STATION")
        
        if secilen_dil == "English":
            p_genel = "18px" 
            p_sifre = "19px"
        else:
            p_genel = "24px" 
            p_sifre = "24px"

        self.ui.label_genelAyarlarYazi.setText(f"<span style='font-size: {p_genel}; font-weight: bold;'>{dil['genel_ayarlar']}</span>")
        self.ui.label_sifre.setText(f"<span style='font-size: {p_sifre}; font-weight: bold;'>{dil['sifre']}</span>")
        self.ui.label_guvenlikKontrolleriYazi.setText(dil["guvenlik_kontrolleri"])
        self.ui.label_kullaniciAdi.setText(dil["kullanici_adi"])
        self.ui.label_telefon.setText(dil["telefon"])
        self.ui.label_canliSistemLog.setText(dil["canli_log"])
        self.ui.label_bildirimler.setText(dil["bildirimler"])
        self.ui.label_sifreleme.setText(dil["sifreleme"])
        self.ui.label_goruntuAyarlari.setText(dil["goruntu_ayarlari"])
        self.ui.label_yayinKalite.setText(dil["yayin_kalitesi"])
        self.ui.label_FPS.setText(dil["fps_siniri"])
        
        durum_klavye = getattr(self, 'klavye_aktif', False)
        self.ui.pushButton_klavye.setText(dil["btn_aktif"] if durum_klavye else dil["btn_pasif"])
        stil_klavye = AYAR_AKTIF if durum_klavye else AYAR_PASIF
        self.ui.pushButton_klavye.setStyleSheet(stil_klavye + " font-size: 16px; font-weight: bold;")
        
        durum_joy = getattr(self, 'joystick_bagli', False)
        self.ui.pushButton_joystick.setText(dil["btn_bagli"] if durum_joy else dil["btn_baglantiyok"])
        self.ui.pushButton_joystick.setStyleSheet(AYAR_AKTIF if durum_joy else AYAR_PASIF)
        
        self.ui.label_veriYedeklemeYazi.setText(dil["veri_yedekleme"])
        self.ui.label_hedefKlasor.setText(dil["hedef_klasor"])
        
        durum_sifre = getattr(self, 'sifreleme_acik', True)
        self.ui.pushButton_sifreleme.setText(dil["btn_acik"] if durum_sifre else dil["btn_kapali"])
        stil_sifre = AYAR_AKTIF if durum_sifre else AYAR_PASIF
        self.ui.pushButton_sifreleme.setStyleSheet(stil_sifre + " font-size: 16px; font-weight: bold;")
        
        self.ui.pushButton_KAYIT.setText(dil["btn_kaydi_durdur"] if getattr(self, 'kayit_yapiyor_mu', False) else dil["btn_kaydet"])
        self.ui.pushButton_simdiYedekle.setText(dil["btn_yedeklemeyi_durdur"] if getattr(self, 'yedekleme_yapiyor_mu', False) else dil["btn_yedekle"])
        self.ui.label_arac.setText(dil["arac_baslik"])
        self.ui.label_yer.setText(dil["yer_baslik"])
        self.ui.label_etap.setText(dil["etap"])

        self.ui.comboBox_temaSecimi.setItemText(0, dil["tema_koyu"])
        self.ui.comboBox_temaSecimi.setItemText(1, dil["tema_acik"])
        self.ui.comboBox_yayinKalite.setItemText(0, dil["kalite_1080"])
        self.ui.comboBox_yayinKalite.setItemText(1, dil["kalite_720"])
        self.ui.comboBox_yayinKalite.setItemText(2, dil["kalite_480"])
        self.ui.comboBox_FPS.setItemText(0, dil["fps_60"])
        self.ui.comboBox_FPS.setItemText(1, dil["fps_30"])

        self.ui.label_sistemBilgisiVeLisans.setText(dil["hakkinda_baslik"])
        self.ui.label_sistemBilgisiVeLisans.setAlignment(Qt.AlignCenter)
        self.ui.textBrowser_Bilgi.setHtml(dil["hakkinda_metin"])

        self.ui.label_navigasyonYazi.setText(dil["nav_baslik"])
        self.ui.label_navigasyonYazi.setAlignment(Qt.AlignCenter)
        
        if secilen_dil == "English":
            self.dinamik_punto_ayarla(self.ui.label_genelAyarlarYazi, 10) 
            self.dinamik_punto_ayarla(self.ui.label_sifre, 11)
            self.dinamik_punto_ayarla(self.ui.pushButton_joystick, 9)
        else: 
            self.dinamik_punto_ayarla(self.ui.label_genelAyarlarYazi, 14) 
            self.dinamik_punto_ayarla(self.ui.label_sifre, 14)
            self.dinamik_punto_ayarla(self.ui.pushButton_joystick, 11)

        try:
            m_metin = self.ui.label_motorYazi.text().upper()
            self.imu_arayuz_guncelle("AKTİF" in m_metin or "ACTIVE" in m_metin)
            g_metin = self.ui.label_GucYazi.text().upper()
            self.guc_arayuz_guncelle("NORMAL" in g_metin)
            gps_metin = self.ui.label_gpsYazi.text().upper()
            self.gps_arayuz_guncelle("ETKİN" in gps_metin or "ACTIVE" in gps_metin)
            k_metin = self.ui.label_kameraYazi.text().upper()
            self.kamera_arayuz_guncelle("HAZIR" in k_metin or "READY" in k_metin)
            l_metin = self.ui.label_lidarYazi.text().upper()
            self.lidar_arayuz_guncelle("TARANIYOR" in l_metin or "SCANNING" in l_metin)
            # Otonom/manuel: metni geri ayristirmak yerine son bilinen gercek
            # durumu kullaniyoruz (daha guvenilir) -- yoksa dil metniyle esler.
            self.mod_durum_guncelle(getattr(self, '_otonom_hazir', False),
                                     getattr(self, '_manuel_hazir', False))
        except Exception as e:
            print(f"Dil Güncelleme Hatası (Ana Sayfa): {e}")

        self.log_yaz(f"Sistem: Dil / Language -> {secilen_dil}")

    def tema_degistir(self, index):
        if index == 1: 
            for cerceve in self.findChildren(QFrame):
                korunacaklar = ["frame_anaKamera", "frame_yanKamera1", "frame_yanKamera2", "frame_yanKamera3", "log_ekran", "frame_BilgiKart"]
                if cerceve.objectName() not in korunacaklar:
                    cerceve.setStyleSheet("") 
            self.setStyleSheet(TEMA_ACIK)
            self.log_yaz("Sistem: Açık Tema Aktif Edildi.")
        elif index == 0: 
            self.setStyleSheet("") 
            for widget in self.findChildren(QWidget):
                if widget.objectName() in self.orijinal_tasarim_hafizasi:
                    widget.setStyleSheet(self.orijinal_tasarim_hafizasi[widget.objectName()])
            self.log_yaz("Sistem: Koyu Tema Aktif Edildi.")

    def dinamik_punto_ayarla(self, eleman, yeni_punto):
        eski_stil = eleman.styleSheet()
        temiz_stil = re.sub(r"font-size\s*:\s*\d+p[tx]\s*;", "", eski_stil)
        eleman.setStyleSheet(temiz_stil + f" font-size: {yeni_punto}pt;")

    def resmi_yuvarla(self, safe_image, kavis, kamera_ismi=""):
        if safe_image is None:
            return QPixmap()
        pixmap = QPixmap.fromImage(safe_image)
        if pixmap.isNull():
            return QPixmap()

        yuvarlak = QPixmap(pixmap.size())
        yuvarlak.fill(Qt.transparent) 
        
        ressam = QPainter(yuvarlak)
        ressam.setRenderHint(QPainter.Antialiasing) 
        
        yol = QPainterPath()
        yol.addRoundedRect(QRectF(yuvarlak.rect()), kavis, kavis)
        ressam.setClipPath(yol)
        ressam.drawPixmap(0, 0, pixmap)

        if kamera_ismi != "": 
            font = QFont("Arial", 18, QFont.Bold)
            ressam.setFont(font)
            ressam.setPen(QColor(0, 0, 0, 200)) 
            ressam.drawText(32, 52, kamera_ismi) 
            ressam.setPen(Qt.red) 
            ressam.drawText(30, 50, kamera_ismi)
        
        ressam.end()
        return yuvarlak
    
    def kamera_sec(self, kamera_id):
        self.aktif_kamera = kamera_id
        self.log_yaz(f"Sistem: Kamera {kamera_id} ana ekrana alındı.")

    def video_ekrana_bas(self, goruntu_sozlugu):
        if not isinstance(goruntu_sozlugu, dict):
            return
        if goruntu_sozlugu.get(1) is not None:
            self.son_kamera_frame_zamani = time.time()
        try:
            # kamera_sistemi.py gercek silah/turret karesini anahtar 1'e,
            # TABELA karesini anahtar 2'ye koyuyor. Tabela tespiti ON kameradan
            # yapiliyor (silah/arka ile alakasi yok) - o yuzden anahtar 2 ON
            # kutusunda gosteriliyor. ARKA kamera donanimi henuz baglanmadi,
            # gercek donanim eklenene kadar bos kalsin.
            mesafe_canli_mi = (time.time() - self.son_hedef_mesafe_zamani) < 1.0
            if mesafe_canli_mi and self.son_hedef_mesafe is not None:
                silah_etiketi = f"{self.kamera_yazi_silah}  {self.son_hedef_mesafe:.1f}m"
            else:
                silah_etiketi = self.kamera_yazi_silah
            pix1 = self.resmi_yuvarla(goruntu_sozlugu.get(1), 48, silah_etiketi)
            pix2 = self.resmi_yuvarla(goruntu_sozlugu.get(2), 48, self.kamera_yazi_on)
            pix3 = self.resmi_yuvarla(None, 48, self.kamera_yazi_arka)

            if not pix1.isNull(): self.tuval_sag1.setPixmap(pix1)
            if not pix2.isNull(): self.tuval_sag2.setPixmap(pix2)
            if not pix3.isNull(): self.tuval_sag3.setPixmap(pix3)
            
            secili_img = goruntu_sozlugu.get(self.aktif_kamera)
            if secili_img:
                buyuk_pix = self.resmi_yuvarla(secili_img, 16, "")
                if not buyuk_pix.isNull():
                    self.tuval_ana.setPixmap(buyuk_pix)
        except Exception as e:
            print(f"Çizim Hatası Detayı: {e}")

    def pwm_sinirini_uygula(self):
        # Eskiden "Telefon" kutusu - artik kolay surus icin PWM ust
        # sinirini (85-255) burada giriyoruz. editingFinished hem Enter'a
        # hem odaktan cikisa tepki verir; QMessageBox modal oldugu icin
        # tekrar tetiklenmeyi onlemek icin basit bir kilit kullaniyoruz.
        if getattr(self, '_pwm_sinir_isliyor', False):
            return
        kutu = self.ui.lineEdit_kullaniciAdi_2
        metin = kutu.text().strip()
        if not metin:
            return

        try:
            deger = float(metin)
        except ValueError:
            QMessageBox.warning(self, "Geçersiz Değer", "Geçersiz sayı. 85 ile 255 arasında bir değer girin.")
            kutu.clear()
            kutu.setStyleSheet(PWM_LIMIT_VARSAYILAN)
            return

        if deger < 85 or deger > 255:
            QMessageBox.warning(self, "Geçersiz Değer", "PWM üst sınırı 85 ile 255 arasında olmalı.")
            kutu.clear()
            kutu.setStyleSheet(PWM_LIMIT_VARSAYILAN)
            return

        self._pwm_sinir_isliyor = True
        try:
            onay = QMessageBox.question(
                self, "PWM Üst Sınırı",
                f"PWM üst sınırı {deger:.0f} olarak ayarlanacak.\n"
                f"(Alt sınır her zaman 85 sabit kalır — motorlar altında dönmüyor.)\n\n"
                f"Onaylıyor musunuz?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes
            )
            if onay == QMessageBox.Yes:
                if hasattr(self, 'telemetri_motoru'):
                    uygulanan = self.telemetri_motoru.pwm_ust_sinirini_ayarla(deger)
                else:
                    uygulanan = deger
                kutu.setText(f"{uygulanan:.0f}")
                kutu.setStyleSheet(PWM_LIMIT_AKTIF)
            else:
                kutu.clear()
                kutu.setStyleSheet(PWM_LIMIT_VARSAYILAN)
        finally:
            self._pwm_sinir_isliyor = False

    def ayar_bildirim_tetikle(self):
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        self.bildirim_acik = not self.bildirim_acik
        self.ui.pushButton_bildirimler.setStyleSheet(AYAR_AKTIF if self.bildirim_acik else AYAR_PASIF)
        self.ui.pushButton_bildirimler.setText(dil["btn_acik"] if self.bildirim_acik else dil["btn_kapali"])

    def ayar_sifreleme_tetikle(self):
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        self.sifreleme_acik = not self.sifreleme_acik
        self.ui.pushButton_sifreleme.setStyleSheet(AYAR_AKTIF if self.sifreleme_acik else AYAR_PASIF)
        self.ui.pushButton_sifreleme.setText(dil["btn_acik"] if self.sifreleme_acik else dil["btn_kapali"])

    def ayar_klavye_tetikle(self):
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        self.klavye_aktif = not getattr(self, 'klavye_aktif', False) 
        if self.klavye_aktif:
            self.ui.pushButton_klavye.setStyleSheet(AYAR_AKTIF) 
            self.ui.pushButton_klavye.setText(dil["btn_aktif"])
            self.log_yaz("Sistem: Klavye Kontrolü (W-A-S-D) AKTİFLEŞTİRİLDİ.")
        else:
            self.ui.pushButton_klavye.setStyleSheet(AYAR_PASIF) 
            self.ui.pushButton_klavye.setText(dil["btn_pasif"])
            self.log_yaz("Sistem: Klavye Kontrolü DEVRE DIŞI.")

    def ayar_joystick_tetikle(self):
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        self.joystick_bagli = not getattr(self, 'joystick_bagli', False)
        if self.joystick_bagli:
            self.ui.pushButton_joystick.setStyleSheet(AYAR_AKTIF)
            self.ui.pushButton_joystick.setText(dil["btn_bagli"])
            self.log_yaz("Donanım: USB Joystick portları taranıyor...")
            if hasattr(self, 'telemetri_motoru'): self.telemetri_motoru.surus_kaynagi = "JOYSTICK"
            if not hasattr(self, 'surus_joystick_motoru'):
                self.surus_joystick_motoru = SurusJoystickThread()
                self.surus_joystick_motoru.pwm_sinyali.connect(self.joystick_pwm_geldi)
                self.surus_joystick_motoru.baglanti_sinyali.connect(self.joystick_baglanti_degisti)
            self.surus_joystick_motoru.start()
        else:
            self.ui.pushButton_joystick.setStyleSheet(AYAR_PASIF)
            self.ui.pushButton_joystick.setText(dil["btn_baglantiyok"])
            self.log_yaz("Donanım: Joystick bağlantısı YAZILIMSAL OLARAK KESİLDİ.")
            if hasattr(self, 'telemetri_motoru'): self.telemetri_motoru.surus_kaynagi = "KLAVYE"
            if hasattr(self, 'surus_joystick_motoru'):
                self.surus_joystick_motoru.durdur()

    def joystick_pwm_geldi(self, sol_pwm, sag_pwm):
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.joystick_pwm_gonder(sol_pwm, sag_pwm)

    def joystick_baglanti_degisti(self, bagli):
        self.log_yaz("🕹️ Joystick Arduino bağlandı." if bagli else "⚠️ Joystick Arduino bağlantısı yok/koptu.")

    def sayfa_odak_ayarla(self, index):
        if index in [0, 1, 2]:
            self.setFocus() 
        else:
            self.clearFocus()

    # ==========================================
    # RADAR ÇEMBERİ + LİDAR NOKTALARI
    # ==========================================
    def radar_cizimi_Guncelle(self):
        self.radar_aci = (self.radar_aci + 2) % 360

        kutu = self.ui.label_radar_cember
        boyut = kutu.width()
        
        if boyut < 10: 
            return 

        tuval = QPixmap(boyut, kutu.height())
        tuval.fill(Qt.transparent)
        
        ressam = QPainter(tuval)
        ressam.setRenderHint(QPainter.Antialiasing)
        
        merkez_x = boyut / 2
        merkez_y = kutu.height() / 2
        
        # 1. DÖNEN KESİK ÇİZGİLİ RADAR ÇEMBERİ ÇİZİMİ
        ressam.translate(merkez_x, merkez_y)
        ressam.rotate(self.radar_aci)
        ressam.translate(-merkez_x, -merkez_y)
        
        kalem = QPen(QColor("#00bfff"))
        kalem.setWidth(4)
        kalem.setStyle(Qt.DashLine)
        ressam.setPen(kalem)
        ressam.drawEllipse(5, 5, boyut - 10, kutu.height() - 10)
        
        # Lidar nokta bulutu KASITLI OLARAK burada çizilmiyor: bu ana menüdeki
        # dekoratif radar çemberi. Gerçek lidar verisi sadece Navigasyon
        # ekranındaki TAKTİK LİDAR panelinde (harita_sistemi.py) gösteriliyor.
        ressam.resetTransform()

        ressam.end()
        kutu.setPixmap(tuval)

    # --- GÜNCELLEME 2: BAĞIMSIZ PALET KLAVYE KONTROLLERİ ---
    def keyPressEvent(self, event):
        if not self.klavye_aktif: return
        if self.ui.stackedWidget.currentIndex() not in [0, 1, 2]: return
        if hasattr(self, 'telemetri_motoru') and self.telemetri_motoru.arac_modu != "MANUEL": return

        if hasattr(self, 'telemetri_motoru'):
            # 1. Yön ve Hareket
            if event.key() == Qt.Key_W:
                self.telemetri_motoru.hareket_emri_gonder("İLERİ (W)")
            elif event.key() == Qt.Key_S:
                self.telemetri_motoru.hareket_emri_gonder("GERİ (S)")
            elif event.key() == Qt.Key_A:
                self.telemetri_motoru.hareket_emri_gonder("SOLA DÖN (A)")
            elif event.key() == Qt.Key_D:
                self.telemetri_motoru.hareket_emri_gonder("SAĞA DÖN (D)")
            elif event.key() == Qt.Key_Space:  
                self.telemetri_motoru.hareket_emri_gonder("DUR (SPACE)")
            
            # 2. Sol Palet Bağımsız Hız Kontrolü (Q: Artır, Z: Azalt)
            elif event.key() == Qt.Key_Q:
                self.telemetri_motoru.hareket_emri_gonder("SOL_HIZ_ARTIR (Q)")
            elif event.key() == Qt.Key_Z:
                self.telemetri_motoru.hareket_emri_gonder("SOL_HIZ_AZALT (Z)")
                
            # 3. Sağ Palet Bağımsız Hız Kontrolü (E: Artır, C: Azalt)
            elif event.key() == Qt.Key_E:
                self.telemetri_motoru.hareket_emri_gonder("SAG_HIZ_ARTIR (E)")
            elif event.key() == Qt.Key_C:
                self.telemetri_motoru.hareket_emri_gonder("SAG_HIZ_AZALT (C)")
                
            # 4. Hızları Eşitleme / Reset (R Tuşu)
            elif event.key() == Qt.Key_R:
                self.telemetri_motoru.hareket_emri_gonder("HIZ_ESITLE (R)")

    def keyReleaseEvent(self, event):
        if not self.klavye_aktif: return
        if self.ui.stackedWidget.currentIndex() not in [0, 1, 2]: return
        if hasattr(self, 'telemetri_motoru') and self.telemetri_motoru.arac_modu != "MANUEL": return

        if event.isAutoRepeat():
            return

        if hasattr(self, 'telemetri_motoru'):
            # Sadece yön tuşlarından el çekilince durmasını sağladık (Hız tuşlarını serbest bıraktık)
            if event.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
                self.telemetri_motoru.hareket_emri_gonder("DUR (SPACE)")

if __name__ == "__main__":
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    pencere = TufanGCS()
    pencere.show()
    sys.exit(app.exec_())