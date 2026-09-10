import sys, os, threading

# --- KENDİ KENDİNİ ONARAN BAŞLATMA (2026-09-05) ---
# Kullanıcı bildirimi: "sen çalıştırdığında imu lidar gözüküyor ben
# çalıştırdığımda gözükmüyor". Canlı doğrulandı: calistir.sh ROS2'yi
# source edip ROS_DOMAIN_ID ayarlıyor, ama kullanıcı main.py'yi DOĞRUDAN
# (kendi terminal alışkanlığı, IDE'nin "Run" tuşu, vb.) başlatınca hiçbiri
# uygulanmıyordu - fresh bir login shell'de "import rclpy" tamamen
# BAŞARISIZ oluyor (canlı test edildi: ModuleNotFoundError), telemetri_
# sistemi.py bunu SESSİZCE devre dışı bırakıyor (ros2_bagli=False) -
# IMU/LIDAR/GPS/sürüş hep "pasif" kalıyor, hiçbir hata görünmüyor. Artık
# main.py, BAŞKA HİÇBİR ŞEY import ETMEDEN ÖNCE rclpy'nin gerçekten
# import edilebilir olup olmadığını kontrol ediyor - edilemiyorsa doğru
# ortamı (calistir.sh ile AYNI) source edip KENDİSİNİ yeniden başlatıyor.
# TUFAN_ENV_HAZIR bayrağı sonsuz döngüyü önlüyor (yeniden başlatma
# SONRASINDA yine olmazsa - ör. ROS2 hiç kurulu değilse - ikinci denemede
# pes edilip mevcut derecelendirilmiş moda düşülüyor, tıpkı eskisi gibi).
if "TUFAN_ENV_HAZIR" not in os.environ:
    try:
        import rclpy  # noqa: F401 - sadece "sourced mi" testi, gerçek kullanım aşağıda
    except ImportError:
        os.environ["TUFAN_ENV_HAZIR"] = "1"
        _bu_dosya = os.path.abspath(__file__)
        _komut = (
            "source /opt/ros/humble/setup.bash 2>/dev/null; "
            "source /home/tufan-yer/ros2_humble/install/setup.bash 2>/dev/null; "
            f"exec python3 {_bu_dosya}"
        )
        os.execvpe("bash", ["bash", "-c", _komut], os.environ)

# --- KRİTİK AĞ DÜZELTMESİ (2026-09-01, v2) ---
# Kullanıcı bildirimi: "ubiquiti bağlayınca arayüz açılmıyor". Canlı teşhis:
# bu makinede WiFi + Ubiquiti + docker0/l4tbr0/usb0/usb1/can0 gibi ilgisiz
# sanal arabirimler AYNI ANDA aktif oluyor. ROS2'nin varsayılan DDS'i
# (Fast-DDS) discovery/gönderim için TÜM arabirimleri kullanmaya çalışıyor -
# bu ANA GUI THREAD'İNİ KERNEL SEVİYESİNDE bloke ediyordu (/proc/PID/wchan:
# sock_alloc_send_pskb'de kilitli kalıyordu - pencere hiç açılmıyor/donuyordu).
# İLK DÜZELTME (v1) SADECE WiFi'yi beyaz listeye almıştı, ama bu YENİ bir
# soruna yol açtı: araç SADECE Ubiquiti'den erişilebilir olduğunda ("WiFi
# yolu Destination Host Unreachable) ROS2 verisi TAMAMEN kesildi ("veri
# gelmiyor", canlı bildirildi) - çalışan tek gerçek yolu yanlışlıkla dışarıda
# bırakmıştık. v2: fastdds_gercek_arabirimler.xml artık HER İKİ gerçek
# arabirimi de (WiFi + Ubiquiti) beyaz listede tutuyor, SADECE ilgisiz
# sanal arabirimleri dışarıda bırakıyor - hangisi çalışıyorsa ROS2 onu
# kullanabilsin diye. Bu env değişkeni rclpy import edilmeden ÖNCE
# (dosyanın en başında) ayarlanmalı - Fast-DDS bunu ilk yüklendiğinde okuyor.
# NOT: WiFi/Ubiquiti IP'leri değişirse fastdds_gercek_arabirimler.xml'deki
# adresler de güncellenmeli (DHCP rezervasyonu/statik IP bu sorunu kalıcı çözer).
def _dds_profilini_guncelle(yol):
    """Beyaz listeyi CANLI IP'lerden yeniden üretir (2026-09-10).

    NEDEN: 10 Eylül'de arayüz açılmadı - pencere geliyor ama boş kalıyor ve
    yanıt vermiyordu ("ekran gelmiyor ama force quit geliyor"). SIGABRT +
    faulthandler ile alınan yığın izi GUI thread'inin tam olarak burada
    kilitlendiğini gösterdi:
        rclpy/node.py:1376 create_subscription
        telemetri_sistemi.py:214 __init__      <- GUI THREAD'İNDE
        main.py:364 _telemetri_baglantilari_kur
    Kök neden: WiFi IP'si DHCP ile 10.40.64.44 -> 192.168.216.150 olmuştu
    ama XML hâlâ ESKİ adresi listeliyordu. Fast-DDS OLMAYAN bir arabirime
    bağlanmaya çalışıp çekirdek seviyesinde bloke oluyordu (aynı dosyanın
    kendi notunda geçen sock_alloc_send_pskb imzası).

    Elle güncelleme talimatı YETMEDİ (XML'de zaten yazıyordu, yine de
    kaçırıldı), o yüzden artık otomatik: her açılışta gerçek arabirimlerin
    IP'leri okunup liste yeniden yazılıyor. Sanal arabirimler (docker0,
    l4tbr0, usb*, can*) DIŞARIDA - onları dahil etmek bu tıkanmanın asıl
    sebebiydi (bkz. XML dosyasının başındaki 2026-09-01 notu).

    Dosya yazılamazsa (salt-okunur disk vb.) sessizce mevcut haliyle
    devam edilir - açılışı ENGELLEMEZ."""
    import io as _io
    import re as _re
    import subprocess as _sp
    try:
        _cikti = _sp.run(["ip", "-4", "-brief", "addr"], capture_output=True,
                         text=True, timeout=5).stdout
    except Exception:
        return
    _adresler = []
    for _satir in _cikti.splitlines():
        _p = _satir.split()
        if len(_p) < 3:
            continue
        _ad = _p[0]
        if _ad == "lo" or _ad.startswith(("docker", "l4tbr", "usb", "can",
                                          "veth", "br-", "tun", "tap")):
            continue
        _ip = _p[2].split("/")[0]
        if _ip:
            _adresler.append((_ad, _ip))
    if not _adresler:
        return          # hiç gerçek arabirim yok - dosyaya dokunma
    _yeni = "\n".join('                    <address>%s</address>  <!-- %s -->'
                      % (_ip, _ad) for _ad, _ip in _adresler)
    _yeni += "\n                    <address>127.0.0.1</address>"
    try:
        _icerik = _io.open(yol, encoding="utf-8").read()
        _yeni_icerik = _re.sub(
            r"(<interfaceWhiteList>\n).*?(\n\s*</interfaceWhiteList>)",
            lambda m: m.group(1) + _yeni + m.group(2),
            _icerik, count=1, flags=_re.S)
        if _yeni_icerik != _icerik:
            _io.open(yol, "w", encoding="utf-8").write(_yeni_icerik)
            print("[DDS] arabirim beyaz listesi guncellendi: "
                  + ", ".join("%s=%s" % (a, i) for a, i in _adresler))
    except Exception as _e:
        print("[DDS] profil guncellenemedi (mevcut haliyle devam): %s" % _e)


if "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
    _dds_profil_yolu = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fastdds_gercek_arabirimler.xml")
    if os.path.isfile(_dds_profil_yolu):
        _dds_profilini_guncelle(_dds_profil_yolu)
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _dds_profil_yolu

# --- KRİTİK AĞ DÜZELTMESİ 2 (2026-09-05) ---
# Kullanıcı bildirimi: "sen çalıştırdığında imu lidar gözüküyor ben
# çalıştırdığımda gözükmüyor" - kök neden: ROS_DOMAIN_ID ayarı sadece
# calistir.sh'de vardı (bkz. o dosyadaki 2026-09-05 notu - izole testle
# KANITLANDI: varsayılan domain 0'da bu paylaşılan ağda saf bir rclpy
# Node() oluşturmak 53 SANİYE sürüyordu, izole bir domainde 0.98 saniye).
# Kullanıcı uygulamayı calistir.sh YERİNE doğrudan (python3 main.py,
# IDE'nin "Run" tuşu, vb.) başlatınca bu ayar hiç uygulanmıyordu - araç
# Jetson domain 77'deyken arayüz varsayılan domain 0'da kalıyor, ikisi
# BİRBİRİNİ HİÇ GÖREMİYORDU (sadece yavaş değil, TAMAMEN kopuk - IMU/
# LIDAR/GPS hep "pasif"). FASTRTPS_DEFAULT_PROFILES_FILE ile AYNI desen:
# rclpy import edilmeden ÖNCE burada, KOD İÇİNDE ayarlanıyor - artık
# başlatma yöntemi (script/IDE/doğrudan terminal) FARK ETMİYOR, HER ZAMAN
# doğru domain kullanılıyor. Araç Jetson'daki tufan_mppi.launch.py de
# AYNI domain'de (77) çalıştırılmalı, aksi halde YİNE görüşemezler.
if "ROS_DOMAIN_ID" not in os.environ:
    os.environ["ROS_DOMAIN_ID"] = "77"

import re, math, time, subprocess, traceback
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QMessageBox, QSizePolicy, QWidget, QFrame, QPushButton, QSpacerItem, QSplitter, QDialog, QGraphicsOpacityEffect, QCheckBox, QGroupBox, QGridLayout
from PyQt5.QtCore import Qt, QRectF, QTimer, QThread, pyqtSignal, QSize, QPropertyAnimation
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPainterPath, QFont, QFontMetrics, QColor, QPen, QIcon
from datetime import datetime

# Yan odalardaki işçileri çağırıyoruz
from kamera_sistemi import KameraThread
from telemetri_sistemi import TelemetriThread
from harita_sistemi import HaritaYoneticisi
from kontrol_paneli_sistemi import KontrolPaneliThread, ayarlari_yukle as _panel_ayarlarini_yukle, ayarlari_kaydet as _panel_ayarlarini_kaydet
from ntrip_rtk_sistemi import NtripRtkThread
from stiller import *
from arayuz import Ui_MainWindow
from diller import CEVIRILER
from terminal_widget import (
    SshTerminalWidget, VARSAYILAN_KULLANICI, UBIQUITI_HEDEF_IP,
)


# ==========================================
# ANA YER KONTROL İSTASYONU SINIFI
# ==========================================
# NOT: Burada eskiden ikinci, tamamen bagimsiz bir "/scan" abonesi
# (LidarThread) vardi - ciktisi (lidar_acilar/lidar_mesafeler) hicbir yerde
# okunmuyordu (dekoratif ana menu radari BILEREK gercek lidar verisi
# cizmiyor, bkz. radar_cizimi_Guncelle). harita_sistemi.py'deki
# Ros2GcsMotoru zaten ayni topic'e abone ve gercekten kullaniliyor - bu
# ikinci, gereksiz ROS2 node/executor/thread'i (ekstra CPU + agi
# yorulmasi) kaldirildi.
class TufanGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # PERFORMANS (2026-09-05, kullanıcı: "lidar ekranında çok fazla
        # komut verisi gecikiyor" - STALL izleyicisi log_yaz'ı da 570ms+
        # takılı yakaladı) - eskiden log_yaz her çağrıda blockCount()>500
        # kontrolü yapıp elle imleç/removeSelectedText ile 100 satır
        # siliyordu (Python seviyesinde ekstra iş). Qt'nin kendi native
        # setMaximumBlockCount() mekanizması AYNI sonucu (en eski satırlar
        # otomatik atılır) C++ tarafında, Python'a hiç çıkmadan yapar -
        # daha hızlı ve log_yaz'daki manuel silme kodunu gereksiz kılar.
        if hasattr(self.ui, 'textEdit_canliSistemLog'):
            self.ui.textEdit_canliSistemLog.document().setMaximumBlockCount(500)

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

        # YENİ (2026-09-01, kullanıcı isteği): "termux gibi bölünmüş
        # ekranlar, hepsini aynı anda görebilmek için" - artık ayrı bir
        # yüzen pencere DEĞİL, doğrudan bu ekranda (Ana Ekran) yan yana
        # bölünen bir QSplitter. "+" butonu da sol bardan KALDIRILDI,
        # terminalin KENDİ küçük araç çubuğuna taşındı (kullanıcı: "artı
        # butonu terminal içinde olsun").
        terminal_konteyner = QWidget(self.ui.log_ekran)
        konteyner_duzen = QVBoxLayout(terminal_konteyner)
        konteyner_duzen.setContentsMargins(0, 0, 0, 0)
        konteyner_duzen.setSpacing(2)

        arac_cubugu = QWidget(terminal_konteyner)
        arac_cubugu.setFixedHeight(28)
        arac_cubugu.setStyleSheet("background-color: #0a0a0a;")
        arac_cubugu_duzen = QHBoxLayout(arac_cubugu)
        arac_cubugu_duzen.setContentsMargins(4, 2, 4, 2)
        arac_cubugu_duzen.addStretch(1)
        yeni_pane_butonu = QPushButton("+ Yeni Terminal", arac_cubugu)
        yeni_pane_butonu.setStyleSheet(
            "QPushButton { background-color: #1a1a1a; color: #00E5FF; border: 1px solid #00E5FF; "
            "border-radius: 4px; padding: 2px 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #00E5FF; color: #000; }"
        )
        yeni_pane_butonu.clicked.connect(self._yeni_terminal_ac)
        arac_cubugu_duzen.addWidget(yeni_pane_butonu)
        # DÜZELTME (2026-09-05, kullanıcı: "arayüz sürekli çöküyor" - kök
        # neden GUI takılma izleyicisiyle tekrar tekrar bu panelin QTextEdit
        # render'ında (bazen 1-6+ SANİYE) bulundu; Qt'nin kuralı gereği bu
        # render GUI thread'İNDEN taşınamıyor). Kullanıcının kendi önerdiği
        # "kaldır" seçeneği: panel gizliyken _ekrani_guncelle TAMAMEN
        # atlanıyor (bkz. terminal_widget.py) - bu buton, kamera/sürüş gibi
        # kritik anlarda paneli devre dışı bırakmayı SEÇENEK olarak sunuyor,
        # ihtiyaç olunca tek tıkla geri getiriliyor (SSH oturumları ARKA
        # PLANDA çalışmaya devam eder, sadece görüntüleme durur).
        gizle_butonu = QPushButton("🔻 Terminali Gizle", arac_cubugu)
        gizle_butonu.setStyleSheet(
            "QPushButton { background-color:#1a1a1a; color:#ffaa00; border:1px solid #ffaa00; "
            "border-radius:4px; padding:2px 10px; font-weight:bold; }"
            "QPushButton:hover { background-color:#ffaa00; color:#000; }"
        )
        gizle_butonu.clicked.connect(lambda: self._terminal_goster_gizle(gizle_butonu))
        arac_cubugu_duzen.addWidget(gizle_butonu)
        konteyner_duzen.addWidget(arac_cubugu)

        self._terminal_splitter = QSplitter(Qt.Horizontal, terminal_konteyner)
        konteyner_duzen.addWidget(self._terminal_splitter, 1)

        self.ui.terminal_ekrani = SshTerminalWidget(self._terminal_splitter)
        self._terminal_splitter.addWidget(self.ui.terminal_ekrani)

        self.ui.gridLayout_4.addWidget(terminal_konteyner, 0, 0, 1, 1)

        # Sayfa her değiştiğinde bu fonksiyonu otomatik çalıştır:
        self.ui.stackedWidget.currentChanged.connect(self.sayfa_odak_ayarla)

        # --- DOKUNMATİK EKRAN KLAVYESİ (onboard) İÇİN HAZIRLIK ---
        # Arayüz normalde gerçek "fullscreen" (X11 _NET_WM_STATE_FULLSCREEN)
        # açılıyordu; bu pencereyi pencere yöneticisinin EN ÜST katmanına
        # koyar ve onboard "force-to-top" (dock) olsa BİLE klavyeyi arkada
        # bırakır (canlı doğrulandı: fullscreen pencere her zaman dock'un
        # üstünde diziliyor). Çözüm: pencereyi çerçevesiz + tam ekran
        # BOYUTUNDA tut ama fullscreen DURUMUNA sokma - böylece onboard'ın
        # dock katmanı arayüzün üstünde kalabiliyor. Klavye açıkken
        # geçici olarak fullscreen'den çıkıyoruz (bkz. ekran_klavyesi_ac_kapat).
        self.setWindowFlags(self.windowFlags() | Qt.FramelessWindowHint)
        self._klavye_gorunur = False
        # DÜZELTME (2026-09-05, kullanıcı: "yine çöktü" - crash_rapor_son.txt
        # ile CANLI YAKALANDI: exit kodu 137 = SIGKILL, kullanıcı/elle
        # kill YOKTU - X11/masaüstü ortamının kendi "yanıt vermiyor"
        # denetimi olduğu düşünülüyor). showFullScreen() eskiden BURADA,
        # aşağıdaki YAVAŞ senkron kurulumlardan (özellikle
        # _telemetri_baglantilari_kur() -> rclpy.init()/Node() - paylaşılan
        # ağda ara sıra 30sn+ sürebiliyor, bkz. ROS_DOMAIN_ID notu) ÖNCE
        # çağrılıyordu - pencere X sunucusuna MAPPED/görünür oluyordu ama
        # Qt olay döngüsü (app.exec_()) HENÜZ BAŞLAMADIĞI için ping/expose
        # gibi pencere yöneticisi protokol mesajlarına CEVAP VEREMİYORDU -
        # tam da bu "yavaş kurulum" sırasında pencere yöneticisi onu
        # "yanıt vermiyor" sayıp öldürmüş olabilir. Artık pencere ancak
        # TÜM yavaş kurulum bittikten SONRA (bu fonksiyonun sonunda)
        # gösteriliyor - o ana kadar hiçbir pencere X sunucusuna hiç
        # MAPPED olmadığı için "yanıt vermiyor" denetimine hiç girmiyor.

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
        self.joystick_bagli = True   # kontrol paneli açılışta otomatik başlar (yazılımsal AÇIK)
        self.klavye_aktif = False
        self.farlar_acik = False
        self.sifreleme_acik = True

        # Kameralar türkçe başlasın
        self.kamera_yazi_on = "ÖN"
        self.kamera_yazi_arka = "ARKA"
        self.kamera_yazi_silah = "SİLAH" 

        self.current_lang = "Türkçe"
        self.aktif_dil = "Türkçe"
        
        self._baslangic_stilleri_uygula()
        self.arayuz_esneklik_ayarlarini_uygula()

        self._telemetri_baglantilari_kur()

        # bkz. yukarıdaki 2026-09-05 notu - TEK yavaş/senkron adım
        # (rclpy.init()/Node() - paylaşılan ağda ara sıra 30sn+ sürebiliyor)
        # burada bitti; pencere ARTIK gösterilebilir. Kasıtlı olarak
        # BURADA (en sonda değil) - aşağıdaki geri kalan kurulum adımları
        # rclpy KULLANMIYOR (kendi Node()'larını hep ayrı thread'lerin
        # run()'ı İÇİNDE oluşturuyorlar, ana thread'i hiç bloklamıyorlar,
        # bkz. DOKUMANTASYON.md) - pencereyi gereksiz yere daha da geç
        # göstermeye gerek yok, sadece GERÇEK riskli adımın ÖNÜNE geçmek
        # yeterli.
        self.showFullScreen()

        self._buton_baglantilarini_kur()
        self._pwm_izleme_butonu_ekle()
        self._panel_testi_butonu_ekle()
        self._arac_baslat_butonu_ekle()
        self._devam_butonu_ekle()
        self._yon_pid_butonu_ekle()
        self._ekran_klavyesi_butonu_ekle()

        self._radar_zamanlayici_baslat()
        self._kamera_ve_joystick_baslat()
        self._kontrol_paneli_baslat()
        self._ntrip_rtk_baslat()

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
        # NOT: eskiden buradaki log, ana ekrandaki terminale yaziliyordu -
        # ama terminal artik GERCEK bir SSH oturumu (ve otomatik PWM akisi),
        # oraya yazi eklemenin bir anlami yok (sadece konsola dusen bos bir
        # uyumluluk fonksiyonu var). Uygulamanin kendi ic olaylari (mod
        # degisimi, kalibrasyon, PWM siniri vb.) artik Ayarlar sayfasindaki
        # "CANLI SİSTEM LOGLARI" kutusuna yaziliyor - onceden hic kullanilmiyordu.
        su_an = datetime.now().strftime("%H:%M:%S")
        satir = f"[{su_an}] {mesaj}"
        print(satir)
        if hasattr(self.ui, 'textEdit_canliSistemLog'):
            # Uzun yarış/test oturumlarında kutunun sınırsız büyümesi
            # artık __init__'te setMaximumBlockCount(500) ile Qt'nin
            # kendi native mekanizmasınca engelleniyor (bkz. yukarısı) -
            # manuel imleç/removeSelectedText kodu KALDIRILDI.
            self.ui.textEdit_canliSistemLog.append(satir)

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
        # Designer'daki sabit "8" yazısı gerçek veri sanılmasın diye - ilk
        # tabela tespiti gelene kadar "-" gösterilir (GPS/LİDAR gibi diğer
        # göstergelerin "PASİF" ile aynı mantık: henüz veri yok = belli olsun).
        self.ui.label_etapNo.setText("-")
        self.telemetri_motoru.imu_durum_sinyali.connect(self.imu_arayuz_guncelle)
        self.telemetri_motoru.guc_durum_sinyali.connect(self.guc_arayuz_guncelle)
        self.telemetri_motoru.gps_durum_sinyali.connect(self.gps_arayuz_guncelle)
        self.telemetri_motoru.kamera_durum_sinyali.connect(self.kamera_arayuz_guncelle)
        self.telemetri_motoru.wifi_durum_sinyali.connect(self.wifi_arayuz_guncelle)
        self.telemetri_motoru.lidar_durum_sinyali.connect(self.lidar_arayuz_guncelle)
        self.telemetri_motoru.mod_durum_sinyali.connect(self.mod_durum_guncelle)
        self.telemetri_motoru.hedef_mesafe_sinyali.connect(self.hedef_mesafe_guncelle)
        self.telemetri_motoru.silah_fazi_sinyali.connect(self._silah_fazi_degisti)
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

        # BEKÇİ (2026-09-04, kullanıcı: "arayüzden veri bazen çok geç
        # gidiyor, arayüzü kapat aç yapınca düzeliyor") - ntrip_rtk_
        # sistemi.py'de daha önce bulunan "zombi thread" ile AYNI kök
        # neden şüphesi: thread isRunning()=True kalırken DDS/rclpy
        # bağlantısı sessizce bozulabiliyor, tek çare tüm uygulamayı
        # kapatıp açmaktı. Artık NTRIP bekçisiyle AYNI desen: her 30sn'de
        # bir gerçek çalışıp çalışmadığı (nabız + isRunning) kontrol
        # edilip, bozulmuşsa SADECE bu thread (tüm uygulama DEĞİL) sessizce
        # yeniden başlatılıyor.
        if not hasattr(self, '_telemetri_bekci_zamanlayici'):
            self._telemetri_bekci_zamanlayici = QTimer()
            self._telemetri_bekci_zamanlayici.setInterval(30000)
            self._telemetri_bekci_zamanlayici.timeout.connect(self._telemetri_bekci_kontrol)
            self._telemetri_bekci_zamanlayici.start()

        # PANEL SÜRECİ BEKÇİSİ (2026-09-06): kontrol paneli artık ayrı bir
        # PROCESS (bkz. _kontrol_paneli_baslat). O süreç herhangi bir
        # sebeple ölürse joystick TAMAMEN sessizce çalışmaz hale gelir -
        # telemetri bekçisiyle AYNI desende, 30sn'de bir yaşayıp
        # yaşamadığına bakılıp gerekirse yeniden başlatılıyor.
        if not hasattr(self, '_panel_bekci_zamanlayici'):
            self._panel_bekci_zamanlayici = QTimer()
            self._panel_bekci_zamanlayici.setInterval(30000)
            self._panel_bekci_zamanlayici.timeout.connect(self._panel_bekci_kontrol)
            self._panel_bekci_zamanlayici.start()

    def _panel_bekci_kontrol(self):
        # Kullanıcı joystick'i BİLEREK kapattıysa (pushButton_joystick)
        # yeniden başlatma - sadece beklenmedik ölümde toparla.
        if not getattr(self, 'joystick_bagli', True):
            return
        sur = getattr(self, '_panel_node_süreci', None)
        if sur is not None and sur.poll() is None:
            return  # bizim başlattığımız süreç sağlıklı çalışıyor
        # Bizim süreç ölmüş olabilir ama SİSTEMDE başka bir node (önceki
        # arayüz oturumundan kalan orphan) hâlâ çalışıyor olabilir - o da
        # joystick'i yayınlıyor, yenisini başlatmaya çalışmak boşuna
        # (tekil-çalışma kilidi zaten engeller). Sadece GERÇEKTEN hiç node
        # yoksa yeniden başlat.
        # DÜZELTME (2026-09-10): burada subprocess.call KULLANILIYORDU -
        # GUI THREAD'İNİ BLOKLAYAN senkron bir çağrı. Yüklü Jetson'da
        # fork+pgrep yüzlerce ms sürebiliyor ve bu bekçi periyodik
        # çalıştığı için arayüz düzenli olarak takılıyordu. STALL
        # izleyicisi bunu canlı yakaladı:
        #     main.py:499 _panel_bekci_kontrol -> subprocess.py:345 call
        # Bu dosyanın kendi kuralı "GUI thread'ini ASLA bloklama" (bkz.
        # _arac_baslat_tetikle'deki Popen notu) - burada ihlal edilmişti.
        # Artık Popen (non-blocking): sonuç BİR SONRAKİ bekçi turunda
        # okunuyor. Bekçi zaten periyodik, bir tur gecikme zararsız.
        _kontrol = getattr(self, '_panel_pgrep_sureci', None)
        if _kontrol is not None:
            _sonuc = _kontrol.poll()
            if _sonuc is None:
                return          # önceki kontrol hâlâ sürüyor, bekle
            self._panel_pgrep_sureci = None
            if _sonuc == 0:
                return          # başka bir örnek çalışıyor, sistem fonksiyonel
        else:
            try:
                self._panel_pgrep_sureci = subprocess.Popen(
                    ["pgrep", "-f", "kontrol_paneli_node.py"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                self._panel_pgrep_sureci = None
            return              # kararı bir sonraki turda ver
        self.log_yaz("⚠️ Kontrol paneli süreci durmuş - yeniden başlatılıyor.")
        self._kontrol_paneli_baslat()

    def _telemetri_bekci_kontrol(self):
        if not hasattr(self, 'telemetri_motoru'):
            return
        olmus_mu = not self.telemetri_motoru.isRunning()
        nabiz_zamani = getattr(self.telemetri_motoru, 'son_nabiz_zamani', 0.0)
        zombi_mi = self.telemetri_motoru.isRunning() and (time.time() - nabiz_zamani) > 90.0
        # DÜZELTME (2026-09-05, kullanıcı: "gps veri yok diyor bunu otomatik
        # düzeltecek bir şey lazım") - yukarıdaki "nabız" kontrolü SADECE
        # executor döngüsünün kendisinin (spin_once) hâlâ dönüp dönmediğini
        # yakalıyor - araç tarafında GPS gerçekten sağlıklı yayınlanırken
        # (canlı defalarca doğrulandı) SADECE bu ABONELİĞİN DDS eşleşmesi
        # bozulup diğer her şey (kamera, telemetri thread'in kendisi) normal
        # çalışmaya devam edebiliyor - "zombi thread" testinden GEÇER ama
        # GPS yine de "veri yok" gösterir. Ayrıca, bağımsız olarak, GPS
        # özelinde de bir tazelik kontrolü ekleniyor - 60sn'den uzun süredir
        # hiç GPSRAW mesajı gelmemişse (araç GPS'siz de olabilir - bu durumda
        # yeniden bağlanmak zararsız, düzelmez ama bozmaz da) bağlantı
        # tazelenir; DDS eşleşmesi bozulmuşsa bu YENİDEN ABONE OLARAK
        # sorunu gerçekten çözer.
        gps_zamani = getattr(self.telemetri_motoru, 'son_gps_zamani', 0.0)
        gps_kesik_mi = (not olmus_mu and not zombi_mi
                        and self.telemetri_motoru.isRunning()
                        and gps_zamani > 0.0
                        and (time.time() - gps_zamani) > 60.0)
        if olmus_mu or zombi_mi or gps_kesik_mi:
            if olmus_mu:
                sebep = "thread durmuş"
            elif zombi_mi:
                sebep = "nabız kesildi (zombi thread)"
            else:
                sebep = "GPS verisi kesildi (abonelik eşleşmesi bozulmuş olabilir)"
            self.log_yaz(f"⚠️ Telemetri bağlantısı {sebep} bulundu - otomatik olarak yeniden başlatılıyor.")
            if zombi_mi:
                # Gerçekten sıkışmış olabilecek eski thread'i GUI'yi
                # DONDURMADAN (wait() ÇAĞIRMADAN) durdurmaya işaretle -
                # kendini kapatamasa bile yeni thread devraliyor (bkz.
                # _ntrip_bekci_kontrol'deki AYNI desen).
                try:
                    self.telemetri_motoru.calisiyor = False
                except Exception:
                    pass
            elif gps_kesik_mi:
                # Zombi DEĞİL (executor sağlıklı dönüyor) - bu yüzden
                # düzgünce durdurup (durdur() -> quit()+wait()) TEMİZ bir
                # yeniden bağlantı yapılabilir, GUI donmaz (çağrı hızlı).
                try:
                    self.telemetri_motoru.durdur()
                except Exception:
                    pass
            self._telemetri_baglantilari_kur()

    def _pwm_izleme_ikonu_olustur(self):
        # arayuz.py OTOMATIK URETILMIS (Qt Designer) oldugu ve elle
        # duzenlenmiyor - bu yuzden yeni bir sidebar butonu icin ikon
        # dosyasi eklemek yerine, mevcut buton ikonlarinin (SVG) rengiyle
        # (#00E5FF) eslesen bir "canli sinyal" ikonu QPainter ile ciziyoruz.
        boyut = 64
        pix = QPixmap(boyut, boyut)
        pix.fill(Qt.transparent)
        ressam = QPainter(pix)
        ressam.setRenderHint(QPainter.Antialiasing)
        kalem = QPen(QColor("#00E5FF"))
        kalem.setWidth(4)
        kalem.setCapStyle(Qt.RoundCap)
        kalem.setJoinStyle(Qt.RoundJoin)
        ressam.setPen(kalem)
        cerceve = QRectF(6, 14, boyut - 12, boyut - 28)
        ressam.drawRoundedRect(cerceve, 8, 8)
        y = cerceve.center().y()
        nabiz = QPainterPath()
        nabiz.moveTo(cerceve.left() + 6, y)
        nabiz.lineTo(cerceve.left() + 15, y)
        nabiz.lineTo(cerceve.left() + 21, y - 13)
        nabiz.lineTo(cerceve.left() + 29, y + 15)
        nabiz.lineTo(cerceve.left() + 37, y - 9)
        nabiz.lineTo(cerceve.left() + 43, y)
        nabiz.lineTo(cerceve.right() - 6, y)
        ressam.drawPath(nabiz)
        ressam.end()
        return QIcon(pix)

    def _yon_pid_ikonu_olustur(self, aktif: bool):
        # Pusula ikonu - AÇIK iken camgobegi (diger ikonlarla ayni renk),
        # KAPALI iken soluk gri (kullaniciya durumu ayirt ettirmek icin).
        boyut = 64
        pix = QPixmap(boyut, boyut)
        pix.fill(Qt.transparent)
        ressam = QPainter(pix)
        ressam.setRenderHint(QPainter.Antialiasing)
        renk = QColor("#00E5FF") if aktif else QColor("#5A6672")
        kalem = QPen(renk)
        kalem.setWidth(4)
        ressam.setPen(kalem)
        merkez_x, merkez_y, r = boyut / 2, boyut / 2, 22
        ressam.drawEllipse(QRectF(merkez_x - r, merkez_y - r, 2 * r, 2 * r))
        ibre = QPainterPath()
        ibre.moveTo(merkez_x, merkez_y - r + 6)
        ibre.lineTo(merkez_x + 7, merkez_y + 6)
        ibre.lineTo(merkez_x, merkez_y)
        ibre.lineTo(merkez_x - 7, merkez_y + 6)
        ibre.closeSubpath()
        ressam.setBrush(renk)
        ressam.drawPath(ibre)
        ressam.end()
        return QIcon(pix)

    def _yon_pid_butonu_ekle(self):
        # Kullanici istegi: manuel duz suruste (klavye VEYA joystick) IMU
        # gyro'suna bakip sapmayi otomatik duzelten bir PID araca eklendi
        # (bkz. arduino_motor_kontrol.py). Once canli veriyle dogrulandi;
        # araç Jetson'inda gyro tam kalibre (mag/accel degil) oldugu icin
        # SADECE gyro kullanilarak tasarlandi - detaylar DOKUMANTASYON.md'de.
        # Varsayilan ACIK (kullanici tercihi) ama sahada beklenmedik
        # davranis olursa tek tikla kapatilabilsin diye buton eklendi.
        buton = QPushButton(self.ui.sol_menu_frame)
        buton.setMinimumSize(QSize(148, 64))
        buton.setMaximumSize(QSize(148, 64))
        buton.setStyleSheet(self.ui.ayarlar_button.styleSheet())
        buton.setText("")
        buton.setCheckable(True)
        buton.setChecked(True)
        buton.setIcon(self._yon_pid_ikonu_olustur(True))
        buton.setIconSize(QSize(64, 64))
        buton.setToolTip("Yön düzeltme PID (AÇIK) - kapatmak için tıkla")
        buton.setObjectName("pushButton_yonPid")
        eklenecek_index = self.ui.verticalLayout_8.indexOf(self.ui.pushButton_pwmIzle)
        self.ui.verticalLayout_8.insertWidget(eklenecek_index, buton)
        buton.clicked.connect(self._yon_pid_degistir)
        self.ui.pushButton_yonPid = buton
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.yon_pid_ayarla(True)

    def _yon_pid_degistir(self):
        aktif = self.ui.pushButton_yonPid.isChecked()
        self.ui.pushButton_yonPid.setIcon(self._yon_pid_ikonu_olustur(aktif))
        self.ui.pushButton_yonPid.setToolTip(f"Yön düzeltme PID ({'AÇIK' if aktif else 'KAPALI'}) - {'kapatmak' if aktif else 'açmak'} için tıkla")
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.yon_pid_ayarla(aktif)
        self.log_yaz(f"🧭 Yön düzeltme PID: {'AÇIK' if aktif else 'KAPALI'}")

    # =====================================================================
    # DOKUNMATİK EKRAN KLAVYESİ (onboard)
    # =====================================================================
    # Kullanıcı isteği (2026-09-03): "direkt dokunmatik ekrana geçtik,
    # masaüstündeki klavyeyi açınca arayüz tam ekran olduğu için klavye
    # arkada kalıyor - arayüzün ÜSTÜNDE açılabilmesi lazım + arayüze uygun
    # bir yere aç/kapat kısayolu."
    #
    # - Klavye programı: onboard (Ubuntu'da kurulu, gsettings'te zaten
    #   force-to-top + alttan dock ayarlı). D-Bus arayüzü:
    #   org.onboard.Onboard  /org/onboard/Onboard/Keyboard  Show/Hide.
    # - Neden fullscreen'den çıkıyoruz: bkz. __init__'teki uzun not.
    # - Buton: sağ ÜST köşede yüzen bir overlay. Sol menüye koymadık çünkü
    #   klavye açıkken (fullscreen değilken) Ubuntu'nun sol dock'u sol menüyü
    #   kısmen örtüyor - sağ üst köşe her iki durumda da erişilebilir.

    def _ekran_klavyesi_ikonu_olustur(self, aktif=False):
        boyut = 36
        pix = QPixmap(boyut, boyut)
        pix.fill(Qt.transparent)
        ressam = QPainter(pix)
        ressam.setRenderHint(QPainter.Antialiasing)
        renk = QColor("#0B0B0B") if aktif else QColor("#E0E0E0")
        kalem = QPen(renk)
        kalem.setWidth(2)
        kalem.setJoinStyle(Qt.RoundJoin)
        ressam.setPen(kalem)
        ressam.setBrush(Qt.NoBrush)
        ressam.drawRoundedRect(QRectF(3, 9, boyut - 6, boyut - 18), 3, 3)
        ressam.setBrush(renk)
        # tuş noktaları (3 sıra)
        for sy in (14, 20):
            for sx in range(8, boyut - 6, 6):
                ressam.drawRect(QRectF(sx, sy, 3, 3))
        # space bar
        ressam.drawRect(QRectF(12, 25, boyut - 24, 3))
        ressam.end()
        return QIcon(pix)

    def _ekran_klavyesi_butonu_ekle(self):
        buton = QPushButton(self.ui.centralwidget)
        buton.setObjectName("pushButton_ekranKlavyesi")
        buton.setCheckable(True)
        buton.setCursor(Qt.PointingHandCursor)
        buton.setFixedSize(56, 56)
        buton.setIcon(self._ekran_klavyesi_ikonu_olustur(False))
        buton.setIconSize(QSize(36, 36))
        buton.setToolTip("Ekran klavyesini aç / kapat")
        buton.setStyleSheet(
            "QPushButton { background-color: rgba(20, 30, 40, 210); "
            "border: 1px solid #444455; border-radius: 12px; }"
            "QPushButton:hover { border: 1px solid #00E5FF; }"
            "QPushButton:checked { background-color: #00E5FF; border: 1px solid #00E5FF; }"
        )
        ekran = QApplication.primaryScreen().geometry()
        buton.move(ekran.width() - buton.width() - 18, 40)
        buton.clicked.connect(self.ekran_klavyesi_ac_kapat)
        buton.show()
        buton.raise_()
        self.ui.pushButton_ekranKlavyesi = buton

    def _onboard_dbus(self, metod):
        # metod: "Show" | "Hide" | "ToggleVisible". onboard çalışmıyorsa
        # dbus-send hata döner -> False (çağıran taraf onboard'u başlatır).
        try:
            sonuc = subprocess.run(
                ["dbus-send", "--type=method_call", "--dest=org.onboard.Onboard",
                 "/org/onboard/Onboard/Keyboard",
                 "org.onboard.Onboard.Keyboard." + metod],
                timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return sonuc.returncode == 0
        except Exception:
            return False

    def ekran_klavyesi_ac_kapat(self):
        self._klavye_gorunur = not self._klavye_gorunur

        if self._klavye_gorunur:
            # 1) Fullscreen KATMANINDAN çık (çerçevesiz + tam ekran boyutunda
            #    kal) ki onboard dock'u üste gelebilsin.
            self.setWindowState(self.windowState() & ~Qt.WindowFullScreen)
            self.setGeometry(QApplication.primaryScreen().geometry())
            self.ui.pushButton_ekranKlavyesi.raise_()
            # 2) Klavyeyi göster; onboard çalışmıyorsa başlat.
            if not self._onboard_dbus("Show"):
                try:
                    subprocess.Popen(["onboard"])
                except FileNotFoundError:
                    self.log_yaz("⌨️ HATA: 'onboard' kurulu değil ->  sudo apt install onboard")
                    self._klavye_gorunur = False
                    self.showFullScreen()
                    self.ui.pushButton_ekranKlavyesi.setChecked(False)
                    return
            self.log_yaz("⌨️ Ekran klavyesi açıldı (arayüz geçici olarak tam ekrandan çıktı).")
        else:
            self._onboard_dbus("Hide")
            self.showFullScreen()
            self.ui.pushButton_ekranKlavyesi.raise_()
            self.log_yaz("⌨️ Ekran klavyesi kapatıldı, arayüz tekrar tam ekran.")

        self.ui.pushButton_ekranKlavyesi.setChecked(self._klavye_gorunur)
        self.ui.pushButton_ekranKlavyesi.setIcon(
            self._ekran_klavyesi_ikonu_olustur(self._klavye_gorunur)
        )

    def _devam_ikonu_olustur(self):
        # Acil durdurma kilidini acan "DEVAM ET" butonu icin yesil oynat/devam
        # oku - diger butonlarla ayni QPainter yontemi (bkz. _pwm_izleme_ikonu_olustur).
        boyut = 64
        pix = QPixmap(boyut, boyut)
        pix.fill(Qt.transparent)
        ressam = QPainter(pix)
        ressam.setRenderHint(QPainter.Antialiasing)
        kalem = QPen(QColor("#00FF7B"))
        kalem.setWidth(4)
        kalem.setJoinStyle(Qt.RoundJoin)
        ressam.setPen(kalem)
        ressam.setBrush(QColor("#00FF7B"))
        merkez_x, merkez_y, r = boyut / 2, boyut / 2, 20
        ressam.drawEllipse(QRectF(merkez_x - r, merkez_y - r, 2 * r, 2 * r))
        ok = QPainterPath()
        ok.moveTo(merkez_x - 7, merkez_y - 11)
        ok.lineTo(merkez_x - 7, merkez_y + 11)
        ok.lineTo(merkez_x + 12, merkez_y)
        ok.closeSubpath()
        kalem2 = QPen(QColor("#0B0B0B"))
        kalem2.setWidth(1)
        ressam.setPen(kalem2)
        ressam.setBrush(QColor("#0B0B0B"))
        ressam.drawPath(ok)
        ressam.end()
        return QIcon(pix)

    def _ates_ikonu_olustur(self):
        # Nisangah/crosshair ikonu - diger butonlarin SVG'leriyle ayni
        # cizim yontemi (QPainter, bkz. _pwm_izleme_ikonu_olustur), ama
        # ACİL DURDUR'un duz kirmizisiyla karismasin diye canlı turuncu.
        boyut = 64
        pix = QPixmap(boyut, boyut)
        pix.fill(Qt.transparent)
        ressam = QPainter(pix)
        ressam.setRenderHint(QPainter.Antialiasing)
        kalem = QPen(QColor("#FF6A00"))
        kalem.setWidth(4)
        kalem.setCapStyle(Qt.RoundCap)
        ressam.setPen(kalem)
        merkez_x, merkez_y, r = boyut / 2, boyut / 2, 16
        ressam.drawEllipse(QRectF(merkez_x - r, merkez_y - r, 2 * r, 2 * r))
        for (x1, y1, x2, y2) in [
            (merkez_x, merkez_y - r - 12, merkez_x, merkez_y - r + 4),
            (merkez_x, merkez_y + r - 4, merkez_x, merkez_y + r + 12),
            (merkez_x - r - 12, merkez_y, merkez_x - r + 4, merkez_y),
            (merkez_x + r - 4, merkez_y, merkez_x + r + 12, merkez_y),
        ]:
            ressam.drawLine(int(x1), int(y1), int(x2), int(y2))
        ressam.setBrush(QColor("#FF6A00"))
        ressam.drawEllipse(QRectF(merkez_x - 3, merkez_y - 3, 6, 6))
        ressam.end()
        return QIcon(pix)

    # ATEŞ (LAZER) YOLU ARTIK BU DOSYADA DEĞİL (2026-09-06): fiziksel sağ
    # toggle switch kontrol paneliyle birlikte AYRI PROCESS'e taşındı, bkz.
    # kontrol_paneli_node.py::ates_degisti + tick(). Eskiden burada 50ms'lik
    # bir _ates_zamanlayici vardı; araç tarafı (turret_node.py,
    # _ATES_MANUEL_TIMEOUT_S = 0.5) tazelenmeyen ateş komutunu yarım saniyede
    # kapattığı için bu tekrar ŞART. Taşıma sırasında zamanlayıcı burada ölü
    # kalmıştı (panel sinyali artık main.py'ye hiç gelmiyor) ve sahada
    # "lazer 1sn sonra sönüyor" olarak görüldü - bu yüzden ölü kod
    # KALDIRILDI, heartbeat node'a taşındı.

    def _arac_baslat_butonu_ekle(self):
        # Kullanici istegi: sol barda PWM butonunun USTUNDE duran "ATEŞ"
        # (lazer) butonu KALDIRILDI (fiziksel sağ toggle switch ile ateş
        # etme AYNEN ÇALIŞMAYA DEVAM EDİYOR, bkz.
        # kontrol_paneli_node.py::ates_degisti) -
        # AYNI KONUMA, araç Jetson'a SSH ile bağlanıp tufan_mppi.launch.py'yi
        # başlatan bir buton kondu (session boyunca elle yapılan "tmux
        # oturumu aç + launch komutu gönder" işlemini tek tıkla yapıyor).
        buton = QPushButton(self.ui.sol_menu_frame)
        buton.setMinimumSize(QSize(148, 64))
        buton.setMaximumSize(QSize(148, 64))
        buton.setStyleSheet(self.ui.ayarlar_button.styleSheet())
        # 2026-09-06 (kullanici istegi: "sol taraftaki aracı başlat butonunu
        # daha güzel bir ikon ile değiştir"): emoji+metin ("🚀 ARACI BAŞLAT")
        # yerine, YANINDAKI butonlarla AYNI yontemle (QPainter ile cizilmis
        # 64x64 QIcon, metinsiz) bir guc/baslatma ikonu. Boylece sol bar
        # gorsel olarak tutarli - digerleri de metinsiz ikon butonlari
        # (bkz. _pwm_izleme_ikonu_olustur, _yon_pid_ikonu_olustur).
        buton.setText("")
        buton.setIcon(self._arac_baslat_ikonu_olustur())
        buton.setIconSize(QSize(64, 64))
        buton.setToolTip("ARAÇ YAZILIMI AÇ/KAPA — çalışmıyorsa başlatır, "
                         "çalışıyorsa durdurur (araç Jetson 192.168.1.22)")
        buton.setObjectName("pushButton_aracBaslat")
        eklenecek_index = self.ui.verticalLayout_8.indexOf(self.ui.pushButton_pwmIzle)
        self.ui.verticalLayout_8.insertWidget(eklenecek_index, buton)
        buton.clicked.connect(self._arac_baslat_onayla)
        self.ui.pushButton_aracBaslat = buton

    def _arac_baslat_ikonu_olustur(self):
        # Guc/baslatma simgesi: ustunde bosluk olan halka + dikey cubuk.
        # Renk #00FF7B - bu projede "olumlu/devam" rengi (bkz. DEVAM
        # mesajlari); yanindaki bilgi butonlarinin camgobegi (#00E5FF)
        # tonundan ve ACİL DURDUR'un kirmizisindan KASITLI olarak ayri,
        # cunku bu buton bir EYLEM baslatiyor. Cizim yontemi digerleriyle
        # ayni (bkz. _pwm_izleme_ikonu_olustur) - arayuz.py Qt Designer
        # tarafindan URETILDIGI icin dosya bazli ikon eklenmiyor.
        boyut = 64
        pix = QPixmap(boyut, boyut)
        pix.fill(Qt.transparent)
        ressam = QPainter(pix)
        ressam.setRenderHint(QPainter.Antialiasing)
        merkez = boyut / 2.0
        r = 18.0
        cerceve = QRectF(merkez - r, merkez - r, 2 * r, 2 * r)
        cubuk_ust = int(merkez - r - 7)
        cubuk_alt = int(merkez - 3)
        # Halkadaki bosluk TAM USTTE olsun diye: 120 dereceden basla, 300
        # derece tara -> geriye 60..120 arasi (tepe noktasi 90) bosluk kalir.
        for kalem in (
            # 1) yumusak dis parilti (yariseffaf, kalin) - "daha guzel"
            #    gorunum icin; ikon 64px'te duz cizgiden daha canli duruyor.
            QPen(QColor(0, 255, 123, 70), 11, Qt.SolidLine, Qt.RoundCap),
            # 2) asil govde
            QPen(QColor("#00FF7B"), 5, Qt.SolidLine, Qt.RoundCap),
        ):
            ressam.setPen(kalem)
            ressam.drawArc(cerceve, 120 * 16, 300 * 16)
            ressam.drawLine(int(merkez), cubuk_ust, int(merkez), cubuk_alt)
        ressam.end()
        return QIcon(pix)

    def _arac_yazilimi_calisiyor_mu(self):
        """Araç yazılımı ayakta mı? goal_manager_node'un heartbeat'i bunun
        GERÇEK işareti (telemetri_sistemi.otonom_heartbeat_cb). Sadece
        onay penceresinin metnini seçmek için kullanılır - ASIL karar
        araç tarafında veriliyor (bkz. _arac_baslat_tetikle), çünkü
        heartbeat gecikmeli/kayıp olabilir ve iki tarafın fikri ayrılırsa
        araçtaki gerçek durum kazanmalı."""
        tm = getattr(self, 'telemetri_motoru', None)
        son = getattr(tm, 'son_otonom_zamani', None) if tm else None
        if not son:
            return False
        return (time.time() - son) < 5.0

    def _arac_baslat_onayla(self):
        # Bu buton ARACIN TÜM sürüş/silah yazılım yığınını başlatıyor -
        # yanlışlıkla dokunmaya karşı (özellikle dokunmatik ekranda, eskiden
        # AYNI KONUMDA "ATEŞ" butonu vardı, refleksle dokunulabilir) basit
        # bir onay isteniyor.
        calisiyor = self._arac_yazilimi_calisiyor_mu()
        if calisiyor:
            baslik, metin = ("Araç Yazılımını DURDUR",
                             "Araç yazılımı ÇALIŞIYOR.\n\n"
                             "Durdurulsun mu? (tufan_mppi.launch.py kapanır,\n"
                             "araç otonom sürüş ve silah kontrolünü kaybeder)")
        else:
            baslik, metin = ("Araç Yazılımını BAŞLAT",
                             "Araç yazılımı çalışmıyor görünüyor.\n\n"
                             "Araç Jetson'a (192.168.1.22) bağlanıp\n"
                             "tufan_mppi.launch.py başlatılsın mı?")
        cevap = QMessageBox.question(self, baslik, metin,
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if cevap == QMessageBox.Yes:
            self._arac_baslat_tetikle()

    def _arac_baslat_tetikle(self):
        # GUI thread'i BLOKE ETMEMEK icin subprocess.Popen (non-blocking) -
        # .wait()/.communicate() KULLANILMIYOR, ssh baglanti/launch suresi
        # (saniyeler) arayuzu asla dondurmuyor. Liste formatinda (shell=True
        # DEGIL) cagriliyor - komut sabit (kullanici girdisi yok), shell
        # injection riski yok. tufan_ana_terminal tmux oturumu YOKSA
        # olusturuluyor, VARSA aynisi kullaniliyor (has-session kontrolü).
        # AÇ/KAPA (2026-09-07, kullanıcı isteği: "başlatma tuşuna
        # bastığımda eğer kodlar çalışıyorsa kapatsın, çalışmıyorsa
        # çalıştırsın"). Karar ARAÇ TARAFINDA veriliyor - tek bir ssh
        # komutu hem kontrol edip hem uyguluyor. Böylece arayüzün
        # (gecikmeli olabilen) heartbeat bilgisi ile araçtaki gerçek
        # durum ayrışsa bile doğru iş yapılır ve GUI thread'i hiç
        # bloke edilmez (Popen, .wait() YOK).
        #
        # *** SAHADA BULUNAN HATA (2026-09-07): buton HEP DURDURUYORDU ***
        # Once desen '[r]os2 launch tufan_v2_ws' idi. Kose parantez
        # numarasi pgrep'in KENDI kabugunu elemesi icin yeterli DEGILDI,
        # cunku ayni komut satirinin ELSE DALINDA baslatma komutu DUZ
        # METIN olarak duruyor ('... && ros2 launch tufan_v2_ws
        # tufan_mppi.launch.py') ve desen ONUNLA eslesiyordu. Sonuc:
        # kabuk her zaman "launch calisiyor" sanip DURDUR dalina giriyor,
        # buton hicbir zaman baslatmiyordu (tmux'ta biriken ^C'ler).
        # COZUM: gercek surecte VAR olan ama bu komutta OLMAYAN bir
        # parcaya bak - gercek surecin komut satiri
        # '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch tufan_v2_ws ...'
        # yani 'bin/ros2 launch' iceriyor; asagidaki metinde ise sadece
        # 'ros2 launch' geciyor, 'bin/' YOK.
        baslat = (
            "tmux has-session -t tufan_ana_terminal 2>/dev/null || "
            "tmux new-session -d -s tufan_ana_terminal; "
            "tmux send-keys -t tufan_ana_terminal "
            "'export ROS_DOMAIN_ID=77 && cd ~/Desktop/tufan_v2_ws && "
            "source /opt/ros/humble/setup.bash && source install/setup.bash && "
            "ros2 launch tufan_v2_ws tufan_mppi.launch.py' Enter"
        )
        durdur = "tmux send-keys -t tufan_ana_terminal C-c"
        uzak_komut = (
            "if pgrep -f '[b]in/ros2 launch tufan_v2_ws' >/dev/null 2>&1; then "
            + durdur + "; else " + baslat + "; fi"
        )
        # *** SAHADA BULUNAN HATA (2026-09-09): buton HİÇ BAŞLATMIYORDU ***
        # pgrep KENDİ KOMUT SATIRINI yakalıyordu. Desen 'bin/ros2 launch
        # tufan_v2_ws' idi ve bu METİN, ssh'ın araçta çalıştırdığı
        # `bash -c "if pgrep -f 'bin/ros2 launch tufan_v2_ws' ..."`
        # sürecinin KOMUT SATIRINDA (pgrep'in ARGÜMANI olarak) duruyordu.
        # pgrep -f tüm komut satırına baktığı için kendi kabuğunu buluyor,
        # kabuk "launch çalışıyor" sanıp DURDUR dalına giriyor, buton
        # hiçbir zaman başlatmıyordu.
        #
        # CANLI KANIT (yığın KAPALIYKEN çalıştırıldı):
        #   pgrep -af 'bin/ros2 launch tufan_v2_ws'
        #   -> 14025 bash -c if pgrep -f 'bin/ros2 launch tufan_v2_ws' ...
        # Yani TEK eşleşme kabuğun KENDİSİ.
        #
        # 2026-09-07'de 'bin/' eklenerek düzeltildiği SANILMIŞTI; işe
        # yaramadı, çünkü yeni desen de aynı komut satırında duruyordu.
        # ÇÖZÜM: köşeli parantez formu - '[b]in/...' regex'i GERÇEK süreçteki
        # 'bin/...' metnini yakalar, ama komutun kendi metni '[b]in/...'
        # olduğu için KENDİNİ yakalamaz. (Bu tuzağa bu projede DÖRDÜNCÜ kez
        # düşüldü: pkill, buton, teşhis komutu, ve butonun 'düzeltmesi'.)
        #
        # ssh çağrısına ayrıca kullanıcı adı (arf203@) açıkça eklendi -
        # ~/.ssh/config'e bağımlı kalmasın - ve stderr bir log dosyasına
        # yazılıyor: Popen non-blocking (.wait() YOK, GUI donmasın) olduğu
        # için ssh hatası başka türlü GÖRÜNMEZ, buton sessizce yalan söyler.
        #
        # Host da artık sabit değil: SSH terminalinin O AN kullandığı host
        # kullanılıyor (Ubiquiti mi WiFi yedeği mi - ikisi AYRIŞMASIN).
        # DİKKAT: _hedef_host_belirle() BURADA ÇAĞRILMAZ - 3 ping denemesi
        # GUI thread'ini ~3 sn dondurur (bkz. bu dosyadaki STALL uyarıları).
        # Terminalin ZATEN çözmüş olduğu değer önbellekten okunur; terminal
        # henüz bağlanmadıysa Ubiquiti IP'si varsayılır (ping YOK).
        term = getattr(self.ui, 'terminal_ekrani', None)
        arac_host = getattr(term, '_son_kullanilan_host', None) or UBIQUITI_HEDEF_IP
        hedef = "%s@%s" % (VARSAYILAN_KULLANICI, arac_host)
        try:
            # stderr bir dosyaya yazılıyor - Popen non-blocking olduğu için
            # (.wait() YOK, GUI donmasın) ssh'ın hatası başka türlü GÖRÜNMEZ;
            # buton "başlatılıyor" der ama hiçbir şey olmazdı (bu hatanın
            # ta kendisi böyle gizlenmişti).
            _ssh_log = open("/tmp/tufan_arac_baslat.log", "a")
            _ssh_log.write("\n--- %s -> %s ---\n"
                           % (time.strftime('%Y-%m-%d %H:%M:%S'), hedef))
            _ssh_log.flush()
            subprocess.Popen(["ssh", "-o", "ConnectTimeout=5",
                              "-o", "BatchMode=yes", hedef, uzak_komut],
                             stdout=_ssh_log, stderr=subprocess.STDOUT)
            self.log_yaz(f"🔗 Araç bağlantısı: {hedef}")
            if self._arac_yazilimi_calisiyor_mu():
                self.log_yaz("🛑 Araç yazılımı DURDURULUYOR...")
            else:
                self.log_yaz("🚀 Araç Jetson'a bağlanılıyor, sürüş yazılımı başlatılıyor...")
        except Exception as e:
            self.log_yaz(f"⚠️ Araç Jetson'a bağlanılamadı: {e}")

    def _panel_testi_butonu_ekle(self):
        # Yer istasyonu fiziksel kontrol panelinin (kontrol_paneli_sistemi.py)
        # HAM degerlerini canli gosteren teshis dialogu - joystick eksen
        # yerlesimi/yonu (SOL_JOY_X_PIN, *_TERS_*) bilinmedigi icin bunlari
        # ayarlarken kullanilir. (Eski "HEDEF: ARAÇ/SİLAH" butonunun yerinde -
        # araç ve silah artik ayri joystick'lerde.)
        buton = QPushButton(self.ui.page_ayarlar)
        buton.setGeometry(QRectF(430, 550, 150, 40).toRect())
        buton.setStyleSheet(self.ui.pushButton_joystick.styleSheet())
        buton.setText("PANEL TESTİ")
        buton.setToolTip("Fiziksel kontrol paneli ham değerlerini canlı göster (eksen/yön ayarı için)")
        buton.setObjectName("pushButton_panelTesti")
        buton.show()
        buton.clicked.connect(self._panel_testi_ac)
        self.ui.pushButton_panelTesti = buton

    def _panel_testi_ac(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Kontrol Paneli Testi")
        dlg.setMinimumSize(460, 620)
        dlg.setStyleSheet("QDialog { background-color: #12181f; }")
        yerlesim = QVBoxLayout(dlg)
        etiket = QLabel("Panel verisi bekleniyor...\n(Arduino bağlı mı? Ayarlar → Joystick AÇIK mı?)")
        etiket.setStyleSheet("color:#E6F7FF; font-family: monospace; font-size: 14px;")
        etiket.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        yerlesim.addWidget(etiket, 1)
        ipucu = QLabel(
            "Her ekseni tek tek oynat, hangi ham kanalın değiştiğini ve yönünü gözle. "
            "Aşağıdaki kutulardan düzelt, KAYDET VE UYGULA'ya bas - anında etkili olur "
            "(yeniden başlatmaya gerek yok). Doğru ayarda: sol joystick ileri → sol_oran "
            "ve sağ_oran birlikte +; sağa çevir → sol_oran > sağ_oran."
        )
        ipucu.setWordWrap(True)
        ipucu.setStyleSheet("color:#8AA8B8; font-size: 11px;")
        yerlesim.addWidget(ipucu)

        # KALİBRASYON AYARLARI (2026-09-04, kullanıcı isteği: "kalibrasyon
        # ayarını arayüze ekleyebilir misin") - eskiden bu sabitler SADECE
        # kontrol_paneli_sistemi.py düzenlenip arayüz yeniden başlatılarak
        # değiştirilebiliyordu. Artık burada, canlı veriye BAKARKEN,
        # kod düzenlemeden değiştirilip anında (yeniden bağlanarak)
        # uygulanabiliyor - bkz. kontrol_paneli_sistemi.py::ayarlari_kaydet/
        # KontrolPaneliThread.ayarlari_yeniden_yukle.
        kalib_kutu = QGroupBox("KALİBRASYON AYARLARI")
        kalib_kutu.setStyleSheet(
            "QGroupBox { color:#00d4ff; font-weight:bold; border:1px solid #1e2836; "
            "border-radius:8px; margin-top:8px; padding-top:6px; }"
            "QGroupBox::title { subcontrol-origin: margin; left:10px; padding:0 4px; }"
            "QCheckBox { color:#E6F7FF; font-size:11px; }"
        )
        kalib_izgara = QGridLayout(kalib_kutu)

        mevcut_ayar = _panel_ayarlarini_yukle()

        cb_sol_pin_degistir = QCheckBox("SOL: X/Y pinlerini değiştir")
        cb_sol_pin_degistir.setChecked(mevcut_ayar["sol_x_pin"] != "A3")
        cb_sol_ters_x = QCheckBox("SOL-X ters çevir")
        cb_sol_ters_x.setChecked(mevcut_ayar["sol_ters_x"])
        cb_sol_ters_y = QCheckBox("SOL-Y ters çevir")
        cb_sol_ters_y.setChecked(mevcut_ayar["sol_ters_y"])
        cb_sol_toggle_ters = QCheckBox("SOL toggle ters kablolu")
        cb_sol_toggle_ters.setChecked(mevcut_ayar["sol_toggle_ters"])

        cb_sag_pin_degistir = QCheckBox("SAĞ: X/Y pinlerini değiştir")
        cb_sag_pin_degistir.setChecked(mevcut_ayar["sag_x_pin"] != "A0")
        cb_sag_ters_x = QCheckBox("SAĞ-X ters çevir")
        cb_sag_ters_x.setChecked(mevcut_ayar["sag_ters_x"])
        cb_sag_ters_y = QCheckBox("SAĞ-Y ters çevir")
        cb_sag_ters_y.setChecked(mevcut_ayar["sag_ters_y"])
        cb_sag_toggle_ters = QCheckBox("SAĞ toggle ters kablolu")
        cb_sag_toggle_ters.setChecked(mevcut_ayar["sag_toggle_ters"])

        kalib_izgara.addWidget(cb_sol_pin_degistir, 0, 0)
        kalib_izgara.addWidget(cb_sol_ters_x, 1, 0)
        kalib_izgara.addWidget(cb_sol_ters_y, 2, 0)
        kalib_izgara.addWidget(cb_sol_toggle_ters, 3, 0)
        kalib_izgara.addWidget(cb_sag_pin_degistir, 0, 1)
        kalib_izgara.addWidget(cb_sag_ters_x, 1, 1)
        kalib_izgara.addWidget(cb_sag_ters_y, 2, 1)
        kalib_izgara.addWidget(cb_sag_toggle_ters, 3, 1)
        yerlesim.addWidget(kalib_kutu)

        kaydet_btn = QPushButton("💾 KAYDET VE UYGULA")
        kaydet_btn.setStyleSheet(
            "QPushButton { background-color:#00994d; color:white; font-weight:bold; "
            "border-radius:6px; padding:8px; }"
            "QPushButton:hover { background-color:#00b359; }"
        )

        def kaydet_ve_uygula():
            yeni_ayar = {
                "sol_x_pin": "A4" if cb_sol_pin_degistir.isChecked() else "A3",
                "sol_ters_x": cb_sol_ters_x.isChecked(),
                "sol_ters_y": cb_sol_ters_y.isChecked(),
                "sol_toggle_ters": cb_sol_toggle_ters.isChecked(),
                "sag_x_pin": "A1" if cb_sag_pin_degistir.isChecked() else "A0",
                "sag_ters_x": cb_sag_ters_x.isChecked(),
                "sag_ters_y": cb_sag_ters_y.isChecked(),
                "sag_toggle_ters": cb_sag_toggle_ters.isChecked(),
            }
            _panel_ayarlarini_kaydet(yeni_ayar)
            # Ayarlar diske yazıldı; ayrı process'teki panel node'u onları
            # yeniden okuyup (yeniden bağlanarak) uygulasın diye komut yolla.
            self._panel_komut_gonder("ayarlari_yeniden_yukle")
            self.log_yaz("🔧 Kontrol paneli kalibrasyonu kaydedildi, yeniden bağlanıyor...")

        kayit_durum_etiketi = QLabel("")
        kayit_durum_etiketi.setStyleSheet("color:#3ddc84; font-weight:bold; font-size:12px;")
        kayit_durum_etiketi.setAlignment(Qt.AlignCenter)

        def kaydet_ve_uygula_ve_bildir():
            kaydet_ve_uygula()
            # Kullanıcı geri bildirimi: "kaydet ve uygula diyince ... kaydedildi
            # diye mesaj gelmiyor oldu mu anlamadım" - eskiden tek bildirim
            # self.log_yaz idi (ana ekrandaki log kutusuna yazılıyordu, bu
            # diyalog ÖNÜNDEYKEN görülmüyordu). Artık diyaloğun İÇİNDE, göz
            # önünde net bir onay yazısı da gösteriliyor. NOT: diyalog
            # KASITLI OLARAK kapanmıyor - canlı ham veriyi izlemeye devam
            # edebilesin diye (bkz. guncelle()).
            kayit_durum_etiketi.setText(
                f"✅ Kaydedildi ve uygulandı ({time.strftime('%H:%M:%S')}) — "
                "panel yeniden bağlanıyor, pencereyi kapatmadan izleyebilirsin."
            )

        kaydet_btn.clicked.connect(kaydet_ve_uygula_ve_bildir)
        yerlesim.addWidget(kaydet_btn)
        yerlesim.addWidget(kayit_durum_etiketi)

        # MERKEZİ SIFIRLA (2026-09-05, canlı bildirildi: "joystickleri
        # ellemiyorum ama panel testinde sağa çekilmiş gibi davranıyor") -
        # güç-açılışı kalibrasyonundaki merkez fiziksel ortadan uzak
        # kalmışsa, canlı oto-merkezleme sınırlı sapmayı (±90 birim) aşan
        # farkı KENDİ BAŞINA düzeltemiyor. Bu buton joystickler BIRAKILMIŞ
        # varsayımıyla aynı güç-açılışı kalibrasyon döngüsünü yeniden
        # çalıştırır (bkz. KontrolPaneliThread.merkezi_sifirla).
        sifirla_btn = QPushButton("🎯 JOYSTICK MERKEZİNİ SIFIRLA")
        sifirla_btn.setStyleSheet(
            "QPushButton { background-color:#cc7a00; color:white; font-weight:bold; "
            "border-radius:6px; padding:8px; }"
            "QPushButton:hover { background-color:#e68a00; }"
        )
        sifirla_ipucu = QLabel(
            "⚠️ Basmadan önce HER İKİ joystick'i de BIRAK (ellemeden ortada dursunlar) - "
            "o anki ham konum yeni merkez (sıfır) olarak kaydedilir."
        )
        sifirla_ipucu.setWordWrap(True)
        sifirla_ipucu.setStyleSheet("color:#e68a00; font-size:11px;")

        def merkezi_sifirla_ve_bildir():
            # (2026-09-06) Panel artık AYRI PROCESS - metot doğrudan
            # çağrılamaz, komut /panel_komut üzerinden gönderilir.
            if self._panel_komut_gonder("merkezi_sifirla"):
                kayit_durum_etiketi.setText(
                    f"🎯 Merkez sıfırlanıyor ({time.strftime('%H:%M:%S')}) — "
                    "joystickleri BIRAKIN, birkaç saniyede yeniden bağlanıp kalibre olacak."
                )
                self.log_yaz("🎯 Joystick merkezi sıfırlama istendi (panel testi).")
            else:
                kayit_durum_etiketi.setText(
                    "⚠️ Kontrol paneli süreci ile bağlantı kurulamadı.")

        sifirla_btn.clicked.connect(merkezi_sifirla_ve_bildir)
        yerlesim.addWidget(sifirla_ipucu)
        yerlesim.addWidget(sifirla_btn)

        def guncelle(d):
            # GERÇEK UYGULANAN PWM (2026-09-04, kullanıcı isteği: "az ittiğimde
            # yavaş gitsin, şu an hep aynı sanırım") - eskiden burada sadece
            # ORAN (-1..1) gösteriliyordu, kullanıcı bunu PWM'e nasıl
            # eşlendiğini göremiyordu; ekrandaki tek sabit sayı (potun
            # belirlediği ÜST SINIR, ör. 151) hep aynı kaldığı için "hiç
            # değişmiyor" sanılıyordu. Şimdi telemetri_motoru._pwm_olcekle
            # ile GERÇEKTEN motorlara giden PWM'i canlı gösteriyoruz - az
            # itince alt sınıra (85) yakın, tam itince üst sınıra (pot
            # değeri) yakın olmalı.
            if hasattr(self, 'telemetri_motoru'):
                gercek_sol_pwm = self.telemetri_motoru._pwm_olcekle(d['sol_oran'])
                gercek_sag_pwm = self.telemetri_motoru._pwm_olcekle(d['sag_oran'])
                pwm_satiri = f"  Gerçek PWM: sol={gercek_sol_pwm:+.0f}   sağ={gercek_sag_pwm:+.0f}   (alt sınır={self.telemetri_motoru.pwm_alt_sinir:.0f}, üst sınır={self.telemetri_motoru.pwm_ust_sinir:.0f})\n"
            else:
                pwm_satiri = ""
            etiket.setText(
                "HAM ANALOG (0-1023)\n"
                f"  Sol joystick  : A3={d['A3']:>4}   A4={d['A4']:>4}\n"
                f"  Sağ joystick  : A0={d['A0']:>4}   A1={d['A1']:>4}\n"
                f"  Potansiyometre: A5={d['A5']:>4}   → PWM {d['pwm']}\n"
                f"  Yer bataryası : A2={d.get('A2')}   → {d.get('yer_bat_metin', 'YOK')}\n\n"
                "HAM DİJİTAL (0 = GND/basılı, 1 = serbest)\n"
                f"  Sol toggle D4={d['D4']}     Sağ toggle D5={d['D5']}\n"
                f"  Sol buton  D9={d['D9']}     Sağ buton  D10={d['D10']}\n\n"
                "HESAPLANAN\n"
                f"  Sürüş : sol_oran={d['sol_oran']:+.2f}   sağ_oran={d['sag_oran']:+.2f}\n"
                f"{pwm_satiri}"
                f"  Silah : turret_x={d['turret_x']:+.2f}   turret_y={d['turret_y']:+.2f}\n"
                f"  Merkez: sol={d['sol_merkez']}   sağ={d['sag_merkez']}"
            )

        # HAM VERİ AKIŞI (2026-09-06): panel ayrı process'te olduğu için
        # ham veri artık /panel_durumu üzerinden {'tip':'ham'} olarak
        # geliyor (bkz. _panel_durumu_geldi). Dialog açıkken node'a
        # "yayınla" komutu gönderilir, kapanınca durdurulur - böylece
        # dialog kapalıyken 50Hz JSON trafiği hiç üretilmez.
        self._panel_ham_guncelle = guncelle
        self._panel_komut_gonder("ham_veri_ac")

        def kapandi(_):
            self._panel_komut_gonder("ham_veri_kapat")
            self._panel_ham_guncelle = None

        dlg.finished.connect(kapandi)
        self._panel_testi_dlg = dlg  # GC'lenmesin
        dlg.show()

    def _pwm_izleme_butonu_ekle(self):
        # Kullanici istegi: terminalde Ctrl+C ile canli PWM akisindan
        # cikildiktan sonra, tek tikla akisa geri donecek bir buton -
        # ana ekrana (terminalin oldugu sayfa) gecip pwm_akisina_don()'u
        # cagirir.
        buton = QPushButton(self.ui.sol_menu_frame)
        buton.setMinimumSize(QSize(148, 64))
        buton.setMaximumSize(QSize(148, 64))
        buton.setStyleSheet(self.ui.ayarlar_button.styleSheet())
        buton.setText("")
        buton.setIcon(self._pwm_izleme_ikonu_olustur())
        buton.setIconSize(QSize(64, 64))
        buton.setToolTip("Canlı PWM akışını göster")
        buton.setObjectName("pushButton_pwmIzle")
        eklenecek_index = self.ui.verticalLayout_8.indexOf(self.ui.pushButton_acilKapat)
        self.ui.verticalLayout_8.insertWidget(eklenecek_index, buton)
        # ACİL DURDUR'a çok yakın durmasın diye araya boşluk. NOT: elle
        # QSpacerItem() olusturup insertItem() ile eklemek PyQt5'te C++
        # tarafinda sahiplik (ownership) sorunu cikarip SEGFAULT'a sebep
        # oldu (canli test edilerek bulundu) - insertSpacing() Qt'nin kendi
        # ic yonetimini kullandigi icin guvenli.
        self.ui.verticalLayout_8.insertSpacing(eklenecek_index + 1, 40)
        buton.clicked.connect(self._pwm_akisini_goster)
        self.ui.pushButton_pwmIzle = buton

    def _pwm_akisini_goster(self):
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.terminal_ekrani.pwm_akisina_don()
        self.ui.terminal_ekrani.setFocus()

    def _terminal_goster_gizle(self, buton):
        # bkz. __init__'teki 2026-09-05 notu - gizlemek SSH oturumlarını
        # KESMEZ (tmux araç Jetson'da yaşamaya devam eder, bkz. terminal_
        # widget.py), sadece bu pahalı GUI render'ını durdurur.
        gizli_mi = self._terminal_splitter.isVisible()
        self._terminal_splitter.setVisible(not gizli_mi)
        buton.setText("🔺 Terminali Göster" if gizli_mi else "🔻 Terminali Gizle")

    def _yeni_terminal_ac(self):
        # Kullanici istegi (2026-09-01, duzeltildi): "termux gibi bölünmüş
        # ekranlar, hepsini aynı anda görebilmek için" VE "artı butonu
        # terminal içinde olsun" - ayrı pencere/panel YOK artık, doğrudan
        # Ana Ekran'daki terminal alanına (bkz. __init__ - terminal_konteyner/
        # self._terminal_splitter) YENİ bir yan-yana pane ekler. Her pane
        # benzersiz isimli AYRI bir tmux oturumuna bağlanır (bkz.
        # terminal_widget.py) - araç kapatılmadan/arayüz kapatılsa BİLE
        # tmux sayesinde çalışmaya devam eder.
        self._yeni_terminal_sayaci = getattr(self, '_yeni_terminal_sayaci', 0) + 1
        oturum_adi = f"tufan_terminal_{self._yeni_terminal_sayaci}"

        # Kapatma butonu eklendi (2026-09-01) - eskiden eklenen pane'i
        # kaldırmanın hiçbir yolu yoktu, yanlışlıkla/deneme amaçlı eklenen
        # pane'ler ekranda kalıcı olarak birikiyordu (canlı bildirildi:
        # "saçma sapan 5 tane yazı"). "×" SADECE bu pane'i görünümden
        # kaldırır (bağlantıyı temiz kapatır) - araçtaki tmux oturumunu
        # ÖLDÜRMEZ, sadece DETACH eder (bkz. terminal_widget.py - kapatma
        # felsefesiyle tutarlı: arayüzden bir şey kapatmak araçta çalışan
        # hiçbir şeyi durdurmamalı).
        pane_konteyner = QWidget(self._terminal_splitter)
        pane_duzen = QVBoxLayout(pane_konteyner)
        pane_duzen.setContentsMargins(0, 0, 0, 0)
        pane_duzen.setSpacing(0)
        pane_cubugu = QWidget(pane_konteyner)
        pane_cubugu.setFixedHeight(22)
        pane_cubugu.setStyleSheet("background-color: #0a0a0a;")
        pane_cubugu_duzen = QHBoxLayout(pane_cubugu)
        pane_cubugu_duzen.setContentsMargins(6, 0, 4, 0)
        etiket = QLabel(oturum_adi, pane_cubugu)
        etiket.setStyleSheet("color: #555; font-size: 10px;")
        pane_cubugu_duzen.addWidget(etiket)
        pane_cubugu_duzen.addStretch(1)
        kapat_butonu = QPushButton("×", pane_cubugu)
        kapat_butonu.setFixedSize(18, 18)
        kapat_butonu.setStyleSheet(
            "QPushButton { background-color: transparent; color: #888; border: none; font-weight: bold; }"
            "QPushButton:hover { color: #ff4444; }"
        )
        pane_cubugu_duzen.addWidget(kapat_butonu)
        pane_duzen.addWidget(pane_cubugu)

        yeni_terminal = SshTerminalWidget(pane_konteyner, tmux_oturum=oturum_adi)
        pane_duzen.addWidget(yeni_terminal, 1)
        kapat_butonu.clicked.connect(lambda: (yeni_terminal.baglantiyi_kapat(), pane_konteyner.deleteLater()))

        self._terminal_splitter.addWidget(pane_konteyner)
        yeni_terminal.setFocus()

    def _devam_butonu_ekle(self):
        # Kullanici istegi: ACİL DURDUR'a basilinca araçtaki motor komutlari
        # kesiliyor (bkz. arduino_motor_kontrol.py _kilitli), bunu geri acmak
        # icin ayri, gorunur bir "DEVAM ET" kisayolu lazim - klavye kisayolu
        # DEGIL, butonlar tercih ediliyor (bkz. pwmIzle butonu gerekcesi).
        # Sadece kilit AKTIFKEN gorunur/tiklanabilir - normal calismada
        # yanlislikla basilamasin ve kilidin o an acik oldugu net olsun.
        buton = QPushButton(self.ui.sol_menu_frame)
        buton.setMinimumSize(QSize(148, 64))
        buton.setMaximumSize(QSize(148, 64))
        buton.setStyleSheet(self.ui.ayarlar_button.styleSheet())
        buton.setText("")
        buton.setIcon(self._devam_ikonu_olustur())
        buton.setIconSize(QSize(64, 64))
        buton.setToolTip("DEVAM ET (acil durdurma kilidini aç)")
        buton.setObjectName("pushButton_devamEt")
        eklenecek_index = self.ui.verticalLayout_8.indexOf(self.ui.pushButton_acilKapat) + 1
        self.ui.verticalLayout_8.insertWidget(eklenecek_index, buton)
        buton.setVisible(False)
        buton.clicked.connect(self._devam_et_tikla)
        self.ui.pushButton_devamEt = buton
        self._arac_kilitli_mi = False

    def _devam_et_tikla(self):
        onay = QMessageBox.question(
            self, "Acil Durdurma Kilidini Aç",
            "Araçtaki motor komutları ACİL DURDUR ile kesildi.\n"
            "Şimdi tekrar sürülebilir hale getirmek istediğinize emin misiniz?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if onay != QMessageBox.Yes:
            return
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.hareket_emri_gonder("DEVAM_CMD")
        self._arac_kilitli_mi = False
        self.ui.pushButton_devamEt.setVisible(False)
        self.ui.terminal_ekrani.append(
            "<br><span style='color: #00FF7B;'><b>✅ DEVAM ET: Acil durdurma kilidi açıldı, araç tekrar komut kabul ediyor.</b></span><br>"
        )
        self.akilli_bildirim_gonder(
            "✅ Devam Ediliyor", "ARAÇTAKİ ACİL DURDURMA KİLİDİ AÇILDI.", kritik_mi=False
        )

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
        # PORT ATAMASI (2026-09-01, kullanici istegi - araç tarafindaki
        # tufan_mppi.launch.py ile BIREBIR AYNI olmali): on(tabela)=5000,
        # silah(turret)=5001, arka=5002 - bkz. kamera_sistemi.py basi.
        self.kamera_motoru = KameraThread(ip="0.0.0.0", on_port=5000, silah_port=5001, arka_port=5002)
        self.kamera_motoru.kare_sinyali.connect(self.video_ekrana_bas)
        self.kamera_motoru.start()

        # KAMERA durumu (2026-09-01, kullanici istegi: "kaç kameranın bağlı
        # olduğunu göstersin" - eskiden TEK bir heartbeat vardı ve SADECE
        # silah kamerasinin karesine bakiyordu, on/arka'nin baglantisiz olup
        # olmadigini hic yansitmiyordu). Artik UC AYRI heartbeat zaman
        # damgasi tutuluyor (bkz. video_ekrana_bas), her biri kendi
        # UDP portundan (5000/5001/5002) gercek kare gelip gelmedigini
        # baglantidan bagimsiz izliyor - veri kesilirse 1.5sn icinde o
        # kamera "bagli degil" sayiliyor.
        self.son_on_frame_zamani = 0.0
        self.son_silah_frame_zamani = 0.0
        self.son_arka_frame_zamani = 0.0
        self.kamera_heartbeat_zamanlayici = QTimer()
        self.kamera_heartbeat_zamanlayici.timeout.connect(self._kamera_heartbeat_kontrol)
        self.kamera_heartbeat_zamanlayici.start(500)

    def _kamera_heartbeat_kontrol(self):
        simdi = time.time()
        bagli_sayisi = sum(1 for zaman in (
            self.son_on_frame_zamani, self.son_silah_frame_zamani, self.son_arka_frame_zamani
        ) if (simdi - zaman) < 1.5)
        self.kamera_arayuz_guncelle(bagli_sayisi)

    def _ntrip_rtk_baslat(self):
        # NTRIP (TUSAGA-Aktif) -> mavros /mavros/gps_rtk/send_rtcm
        # koprusu (bkz. ntrip_rtk_sistemi.py).
        self.ntrip_motoru = NtripRtkThread()
        self.ntrip_motoru.log_sinyali.connect(self.log_yaz)
        self.ntrip_motoru.start()

        # BEKÇİ (2026-09-01, canlı bulundu - kritik güvenilirlik açığı):
        # bu thread bazen SESSİZCE ölüyordu (ROS2 düğümü kayboluyordu,
        # ana uygulama sorunsuz çalışmaya devam ediyordu, kullanıcı
        # "RTK neden Fixed'e girmiyor" diye sorana kadar fark edilmedi -
        # aslında düzeltme akışı tamamen durmuştu). Artık her 30 saniyede
        # bir thread'in gerçekten çalışıp çalışmadığı kontrol ediliyor -
        # ölmüşse otomatik olarak YENİDEN başlatılıyor (goal_manager_
        # watchdog.py'nin araç tarafında yaptığı AYNI bekçi deseni).
        self._ntrip_bekci_zamanlayici = QTimer()
        self._ntrip_bekci_zamanlayici.setInterval(30000)
        self._ntrip_bekci_zamanlayici.timeout.connect(self._ntrip_bekci_kontrol)
        self._ntrip_bekci_zamanlayici.start()

    def _ntrip_bekci_kontrol(self):
        if not hasattr(self, 'ntrip_motoru'):
            return
        # DÜZELTME (2026-09-01, canlı bulundu - "rtk verisini gönderen node
        # mu çöküyor" sorusu araştırılırken): SADECE isRunning() kontrolü
        # KÖR NOKTA - canlı bir vakada thread isRunning()=True kalmaya
        # devam ederken ROS düğümü (`tufan_ntrip_rtk_koprusu`) DDS
        # grafiğinden tamamen kayboldu, run()'daki try/except HİÇBİR hata
        # loglamadı (kod incelemesinde açık bir destroy_node()/shutdown()
        # çağrısı da bulunamadı - "zombi" thread: canlı görünüyor ama
        # düzeltme akışı fiilen durmuş). Artık ntrip_rtk_sistemi.py'nin
        # her döngü turunda güncellediği `son_nabiz_zamani`nın tazeliği de
        # kontrol ediliyor - ikisinden biri (thread bitmiş VEYA nabız
        # STALE_ESIK_SN'den (75sn) fazla bayatlamış) yeniden başlatmayı
        # tetikliyor.
        olmus_mu = not self.ntrip_motoru.isRunning()
        nabiz_zamani = getattr(self.ntrip_motoru, 'son_nabiz_zamani', 0.0)
        zombi_mi = self.ntrip_motoru.isRunning() and (time.time() - nabiz_zamani) > 90.0
        if olmus_mu or zombi_mi:
            sebep = "thread durmuş" if olmus_mu else "nabız kesildi (zombi thread)"
            self.log_yaz(f"⚠️ NTRIP/RTK {sebep} bulundu - otomatik olarak yeniden başlatılıyor.")
            if zombi_mi:
                # Gerçekten sıkışmış olabilecek eski thread'i GUI'yi
                # DONDURMADAN (wait() ÇAĞIRMADAN) durdurmaya işaretle -
                # kendini kapatamasa bile yeni thread devraliyor.
                try:
                    self.ntrip_motoru._calisiyor = False
                except Exception:
                    pass
            self.ntrip_motoru = NtripRtkThread()
            self.ntrip_motoru.log_sinyali.connect(self.log_yaz)
            self.ntrip_motoru.start()

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
        durum = ("ACTIVE" if aktif_mi else "PASSIVE") if dil == "English" else ("AKTİF" if aktif_mi else "PASİF")
        self.ui.label_motorYazi.setText(f"IMU : {durum}")
        self.ui.label_motorYazi.setStyleSheet(f"color: {'#00ff00' if aktif_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def guc_arayuz_guncelle(self, normal_mi):
        dil = getattr(self, 'current_lang', 'Türkçe')
        baslik = "POWER SYSTEM" if dil == "English" else "GÜÇ SİSTEMİ"
        durum = ("NORMAL" if normal_mi else "CRITICAL") if dil == "English" else ("NORMAL" if normal_mi else "KRİTİK")
        self.ui.label_GucYazi.setText(f"{baslik} : {durum}")
        self.ui.label_GucYazi.setStyleSheet(f"color: {'#00ff00' if normal_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def gps_arayuz_guncelle(self, etkin_mi):
        dil = getattr(self, 'current_lang', 'Türkçe')
        durum = ("ACTIVE" if etkin_mi else "PASSIVE") if dil == "English" else ("ETKİN" if etkin_mi else "PASİF")
        self.ui.label_gpsYazi.setText(f"GPS : {durum}")
        self.ui.label_gpsYazi.setStyleSheet(f"color: {'#00ff00' if etkin_mi else 'red'}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")
    
    def wifi_arayuz_guncelle(self, yuzde, kaynak="UBIQUITI"):
        # Ubiquiti nokta-nokta linkinin baglanti kalitesi - Ubiquiti tamamen
        # kopukken (%0) telemetri_sistemi.py otomatik olarak WiFi'yi dener
        # ve kaynagi "WIFI" olarak bildirir, gösterge o zaman WIFI yazar
        # (bkz. telemetri_sistemi.py _wifi_kontrol).
        self.ui.label_wifiYazi.setText(f"{kaynak} : %{yuzde}")
        if yuzde > 60:
            self.ui.label_wifiYazi.setStyleSheet("color: #00ff00; font-weight: bold; font-size: 24px; border: none; background-color: transparent;") 
        elif yuzde > 30:
            self.ui.label_wifiYazi.setStyleSheet("color: orange; font-weight: bold; font-size: 24px; border: none; background-color: transparent;") 
        else:
            self.ui.label_wifiYazi.setStyleSheet("color: red; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

    def kamera_arayuz_guncelle(self, bagli_sayisi):
        # DÜZELTME (2026-09-01, kullanıcı isteği): eskiden tek bir AKTİF/
        # BAĞLANTI KOPTU metniydi (SADECE silah kamerasına bakıyordu, bkz.
        # _kamera_heartbeat_kontrol notu). Artık 3 kameradan (ön/silah/arka)
        # KAÇININ gerçekten bağlı olduğunu gösteriyor - "2/3 BAĞLI" gibi.
        # Hiçbiri bağlı değilse eski/tanıdık "BAĞLANTI KOPTU" metni korunuyor.
        dil = getattr(self, 'current_lang', 'Türkçe')
        baslik = "CAMERA" if dil == "English" else "KAMERA"
        if bagli_sayisi <= 0:
            durum = "NO CONNECTION" if dil == "English" else "BAĞLANTI KOPTU"
            renk = "red"
        elif bagli_sayisi >= 3:
            durum = "3/3 CONNECTED" if dil == "English" else "3/3 BAĞLI"
            renk = "#00ff00"
        else:
            durum = f"{bagli_sayisi}/3 CONNECTED" if dil == "English" else f"{bagli_sayisi}/3 BAĞLI"
            renk = "#ffaa00"  # kısmi bağlantı - ne tam yeşil ne kırmızı
        self.ui.label_kameraYazi.setText(f"{baslik} : {durum}")
        self.ui.label_kameraYazi.setStyleSheet(f"color: {renk}; font-weight: bold; font-size: 24px; border: none; background-color: transparent;")

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
                durum, renk = otonom_metni, "#00ff00"
            else:
                durum, renk = manuel_metni, "#00ff00"
        elif self._otonom_hazir:
            durum, renk = otonom_metni, "#00ff00"
        elif self._manuel_hazir:
            durum, renk = manuel_metni, "#00ff00"
        else:
            durum, renk = ("PASSIVE" if dil == "English" else "PASİF"), "red"

        metin = f"{baslik} : {durum}"
        self.ui.label_sistemYazi.setText(metin)
        self.ui.label_sistemYazi.setStyleSheet(f"color: {renk}; font-weight: bold;")
        self._sistem_yazisini_sigdir(metin)

    def _sistem_yazisini_sigdir(self, metin):
        # "SİSTEM : OTONOM HAZIR" gibi uzun metinler sabit 32px fontla
        # kutudan tasip iki uctan kirpiliyordu (kutu genisligi sabit,
        # AlignCenter). Metin uzunlugu degistikce (dil, mod) font boyutunu
        # kutuya sigacak sekilde otomatik kucultuyoruz.
        # NOT: label_sistemYazi'nin kendi kutusu (371px) daireye (frame_halka,
        # 400px çap) neredeyse tam sığacak şekilde yerleştirilmiş, ama
        # metnin bulunduğu yükseklikte dairenin GERÇEK (yuvarlak sınırdan
        # dolayı kutudan dar) genişliği kutunun kendisinden bile az farkla
        # dar - eski 12px kenar payı yetersizdi, metin dairenin dışına az
        # da olsa taşıyordu (canlı gözlemlendi). Daha güvenli pay veriyoruz.
        font = self.ui.label_sistemYazi.font()
        kutu_genisligi = self.ui.label_sistemYazi.width() - 50  # kenar payi (daire sinirina gore guvenli)
        boyut = 32
        while boyut > 14:
            font.setPixelSize(boyut)
            if QFontMetrics(font).horizontalAdvance(metin) <= kutu_genisligi:
                break
            boyut -= 1
        self.ui.label_sistemYazi.setFont(font)

    def manuel_sec(self):
        # Operatör MANUEL'e bastıysa ve araç tarafında silah fazı
        # sürüyorsa fazı KES (bkz. telemetri_sistemi.silah_fazini_iptal_et).
        #
        # *** _silah_fazi_mod_degisimi BAYRAĞI ŞART ***: silah fazı
        # BAŞLARKEN aracı MANUEL'e alan çağrı da buraya düşüyor ve o an
        # telemetri_motoru.silah_fazi_aktif ZATEN True (silah_fazi_cb
        # bayrağı sinyalden ÖNCE set ediyor). Bu koruma olmadan faz
        # başlar başlamaz KENDİNİ İPTAL EDERDİ - kod incelemesinde
        # yakalandı, sahaya çıkmadan önce.
        tm = getattr(self, 'telemetri_motoru', None)
        if (tm is not None and getattr(tm, 'silah_fazi_aktif', False)
                and not getattr(self, '_silah_fazi_mod_degisimi', False)):
            tm.silah_fazini_iptal_et()
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
        self._arac_kilitli_mi = True
        if hasattr(self.ui, 'pushButton_devamEt'):
            self.ui.pushButton_devamEt.setVisible(True)

    def veri_kaydet_tetikle(self):
        """KAYIT butonu - SADECE KAMERA GÖRÜNTÜLERİNİ kaydeder (2026-09-09,
        kullanıcı isteği: "veri kaydet butonuna basınca sadece kamera
        görüntülerini kaydetsin"). Telemetri/log/harita HİÇBİR ŞEY bu
        dosyalara yazılmaz.

        SAHADA BULUNAN HATA: bu metot `kamera_motoru.kayit_durumu_degistir()`
        çağırıyordu ama KameraThread'de ÖYLE BİR METOT YOKTU - buton yazı ve
        rengini değiştiriyor, sonra AttributeError alıyordu. Yani buton
        "KAYIT" görünüyor ama diske TEK KARE yazmıyordu. Metot artık
        kamera_sistemi.py'de gerçekten var (bkz. KameraThread.
        kayit_durumu_degistir) ve üç kamerayı ayrı ayrı .avi dosyalarına
        yazıyor.
        """
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        if not self.kayit_yapiyor_mu:
            klasor = None
            if hasattr(self, 'kamera_motoru'):
                try:
                    klasor = self.kamera_motoru.kayit_durumu_degistir(True)
                except Exception as e:
                    self.log_yaz(f"⚠️ Kayıt başlatılamadı: {e}")
                    return
            self.ui.pushButton_KAYIT.setStyleSheet(KAYIT_AKTIF)
            self.ui.pushButton_KAYIT.setText(dil["btn_kaydi_durdur"])
            self.kayit_yapiyor_mu = True
            self.log_yaz("🎥 KAMERA KAYDI BAŞLADI"
                         + (f" -> {klasor}" if klasor else ""))
        else:
            klasor = None
            if hasattr(self, 'kamera_motoru'):
                try:
                    klasor = self.kamera_motoru.kayit_durumu_degistir(False)
                except Exception as e:
                    self.log_yaz(f"⚠️ Kayıt durdurulamadı: {e}")
            self.ui.pushButton_KAYIT.setStyleSheet(KAYIT_PASIF)
            self.ui.pushButton_KAYIT.setText(dil["btn_kaydet"])
            self.kayit_yapiyor_mu = False
            self.log_yaz("⏹️ KAMERA KAYDI DURDU"
                         + (f" -> {klasor}" if klasor else ""))

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

        # NTRIP/RTK istemcisine gercek GNSS konumunu ilet (AUTO/VRS
        # mountpoint dogru referans istasyonunu secebilsin diye).
        if hasattr(self, 'ntrip_motoru'):
            self.harita_yoneticisi.ros_motoru.gnss_sinyal.connect(
                self.ntrip_motoru.guncel_konum_ayarla)

        # KONTROL PANELİ GÖSTERGELERİ (2026-09-06) - panel artık ayrı bir
        # process'te okunuyor (bkz. _kontrol_paneli_baslat); komutları O
        # yayınlıyor, arayüz sadece göstergeleri buradan güncelliyor.
        self.harita_yoneticisi.ros_motoru.panel_durum_sinyali.connect(
            self._panel_durumu_geldi)

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
            # DÜZELTME (2026-09-01): eskiden eski metni ("HAZIR" var mı diye)
            # geri ayrıştırıyordu - artık "X/3 BAĞLI" bir sayı taşıdığı için
            # bu metin-kazıma güvenilir değil. Doğrudan canlı zaman
            # damgalarından YENİDEN hesaplanıyor (daha doğru, dil bağımsız).
            if hasattr(self, '_kamera_heartbeat_kontrol'):
                self._kamera_heartbeat_kontrol()
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
        if safe_image is None or safe_image.isNull():
            return QPixmap()
        # DÜZELTME (2026-09-05, kullanıcı: "kamera görüntüsü küçültme
        # yapma") - eskiden burada manuel bir ön-küçültme (QImage.scaled)
        # yapılıyordu; GUI takılma izleyicisiyle bunun GIL çekişmesi
        # yüzünden bazen 1200ms'ye kadar tıkandığı görüldü. Zaten gereksiz
        # bir işti: tuval_ana/sag1-3 hepsi setScaledContents(True) ile
        # kuruldu (main.py::__init__) - Qt, pixmap'i widget boyutuna KENDİ
        # native (C++ tarafında, Python'dan çok daha ucuz) ölçekleme
        # motoruyla zaten sığdırıyor, ayrıca bizim elle küçültmemize hiç
        # gerek yoktu.
        #
        # QPixmap.fromImage() + drawPixmap yerine drawImage() DOĞRUDAN
        # çiziyor - Qt forum kaynaklarında QPixmap dönüşümünün kendisinin
        # yavaş olduğu (1280x960'da ~30ms) belgelendi, ara adım atlanıyor.
        yuvarlak = QPixmap(safe_image.size())
        yuvarlak.fill(Qt.transparent)

        ressam = QPainter(yuvarlak)
        ressam.setRenderHint(QPainter.Antialiasing)

        yol = QPainterPath()
        yol.addRoundedRect(QRectF(yuvarlak.rect()), kavis, kavis)
        ressam.setClipPath(yol)
        ressam.drawImage(0, 0, safe_image)

        if kamera_ismi != "":
            # bu fonksiyon saniyede onlarca kez çağrılıyor (kamera akışı),
            # her seferinde YENİ bir QFont nesnesi yaratmanın (font arama/
            # eşleme her defasında tekrar yapılır) bir anlamı yok - tek
            # sabit stil, bir kez oluşturup sakla.
            if not hasattr(self, '_kamera_etiket_fontu'):
                self._kamera_etiket_fontu = QFont("Arial", 18, QFont.Bold)
            ressam.setFont(self._kamera_etiket_fontu)
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
        # GEÇİCİ ÖLÇÜM (2026-09-04, "kesin sebebi bul") - bu SLOT'un
        # GUI thread'e GERÇEKTE ne sıklıkla ulaştığını ölçüyoruz
        # (resmi_yuvarla'nın İÇ maliyetinden BAĞIMSIZ) - eğer aralık
        # beklenenden (kamera ~66ms) çok daha büyükse, GUI thread'in
        # BAŞKA bir şeyle meşgul kalıp kareyi geç işlediği kanıtlanır.
        _cagri_t = time.perf_counter()
        self._video_cagri_gecmisi = getattr(self, '_video_cagri_gecmisi', [])
        son = getattr(self, '_son_video_cagri_t', None)
        if son is not None:
            self._video_cagri_gecmisi.append((_cagri_t - son) * 1000.0)
        self._son_video_cagri_t = _cagri_t
        if len(self._video_cagri_gecmisi) >= 60:
            gecmis = self._video_cagri_gecmisi
            ort = sum(gecmis) / len(gecmis)
            en_kotu = max(gecmis)
            print(f"[PERF slot-aralığı] n={len(gecmis)}  ort={ort:.1f}ms  "
                  f"en_kötü={en_kotu:.1f}ms  (beklenen ~66ms - üstü GUI "
                  f"thread'in BAŞKA işle meşgul olduğunu gösterir)")
            self._video_cagri_gecmisi = []
        # HER ÜÇ kamera için AYRI heartbeat (2026-09-01, bkz. _kamera_ve_
        # joystick_baslat notu) - hangi sayfada olunursa olunsun HER ZAMAN
        # güncellenir, "kaç kamera bağlı" göstergesi sayfadan bağımsız doğru
        # kalsın diye (aşağıdaki erken dönüşten ÖNCE, tıpkı eskisi gibi).
        simdi = time.time()
        if goruntu_sozlugu.get(1) is not None:
            self.son_silah_frame_zamani = simdi
        if goruntu_sozlugu.get(2) is not None:
            self.son_on_frame_zamani = simdi
        if goruntu_sozlugu.get(3) is not None:
            self.son_arka_frame_zamani = simdi
        # PERFORMANS: kamera onizlemelerini yeniden cizmek (4x QPainter/
        # resmi_yuvarla) her karede pahali. Kamera sayfasinda degilken
        # (indeks 1) bu cizimi atlayip diger ekranlarda "kasma" yaratmasini
        # onluyoruz - kalp atisi (yukarida) yine de guncel kaliyor.
        # DÜZELTME (2026-09-02, kullanıcı isteği: "suni ufuk kısmına silah
        # kamerasının görüntüsünü koyalım... jetsonun kasmaması için zaten
        # bir ekran açılınca diğerini kapatma olayı vardı") - kullanıcının
        # kendi hatırlattığı AYNI kalıp genişletildi: Navigasyon sayfasında
        # (indeks 2) SADECE silah karesi Suni Ufuk'a iletilir (rounded-rect
        # işlemesi YOK - Suni Ufuk zaten kendi kırpmasını/dönüşümünü kendi
        # paintEvent'inde yapıyor, bkz. SuniUfukEkrani - burada SADECE
        # QImage->QPixmap çevrimi, Kamera sayfasının 4x resmi_yuvarla/
        # QPainter bindirmesinden ÇOK daha hafif) - HİÇBİR ZAMAN ikisi
        # (Kamera sayfasının tam bindirmesi + Navigasyon'un Suni Ufuk
        # güncellemesi) AYNI ANDA çalışmaz, sadece o an GÖRÜNEN sayfanınki
        # çalışır.
        # DÜZELTME (2026-09-04, kullanıcı: "hala gecikme var arayüzde
        # kasmalar var") - backlog boşaltma (bkz. kamera_sistemi.py)
        # gecikmenin BİRİKMESİNİ önledi ama asıl "kasma" hissinin kaynağı
        # BAŞKAYDI: her GELEN kare (3 kamera birden, kameranın kendi FPS'i
        # kadar - saniyede onlarca kez) GUI thread'inde pahalı bir işi
        # (Kamera sayfasında 4x QPainter/resmi_yuvarla, Navigasyon'da Suni
        # Ufuk'un tam yeniden çizimi) TETİKLİYORDU - gelen kare hızı ekran
        # yenileme hızından kat kat fazla olduğu için GUI thread'i sürekli
        # meşgul kalıp TÜM arayüzü (sadece kamerayı değil) kasıyordu. Artık
        # bu ağır işler saniyede en fazla ~15 kez (66ms) yapılıyor - ARADA
        # kalan kareler sessizce atlanıyor (kalp atışı takibi HALA HER
        # KAREDE güncelleniyor, yukarıda - "bağlı/kopuk" göstergesi bundan
        # etkilenmiyor, sadece EKRANA ÇİZME hızı sınırlanıyor).
        simdi_render = time.time()
        if simdi_render - getattr(self, '_son_video_render_zamani', 0.0) < 0.066:
            return
        self._son_video_render_zamani = simdi_render

        aktif_sayfa = self.ui.stackedWidget.currentIndex()
        if aktif_sayfa == 2 and hasattr(self, 'harita_yoneticisi'):
            silah_img = goruntu_sozlugu.get(1)
            if silah_img is not None:
                pix = QPixmap.fromImage(silah_img)
                if not pix.isNull():
                    self.harita_yoneticisi.silah_kamera_guncelle(pix)
            return
        if aktif_sayfa != 1:
            return
        try:
            # kamera_sistemi.py gercek silah/turret karesini anahtar 1'e,
            # ON (tabela) karesini anahtar 2'ye, ARKA karesini anahtar 3'e
            # koyuyor (port atamasi: on=5000, silah=5001, arka=5002 - bkz.
            # kamera_sistemi.py basi). Tabela tespiti ON kameradan yapiliyor
            # (silah/arka ile alakasi yok) - o yuzden anahtar 2 ON kutusunda
            # gosteriliyor. DUZELTME (2026-09-01, kullanici istegi): ARKA
            # artik SABIT None DEGIL - gercek arka_kamera_node.py karesi.
            mesafe_canli_mi = (time.time() - self.son_hedef_mesafe_zamani) < 1.0
            if mesafe_canli_mi and self.son_hedef_mesafe is not None:
                silah_etiketi = f"{self.kamera_yazi_silah}  {self.son_hedef_mesafe:.1f}m"
            else:
                silah_etiketi = self.kamera_yazi_silah
            # DÜZELTME (2026-09-04, kullanıcı: "küçük kamera görüntüsü
            # akmasın sadece büyütülen kamera çalışsın") - 3 küçük önizleme
            # kutusu (tuval_sag1/2/3) her tikte (yukarıdaki throttle'a
            # rağmen hâlâ ~15Hz) resmi_yuvarla ile yeniden çiziliyordu -
            # sadece BÜYÜTÜLEN (seçili) kamera canlı akış ihtiyacındayken
            # bu üçü de GUI thread'ini gereksiz yere meşgul ediyordu.
            # Artık küçük kutular ÇOK daha yavaş (saniyede ~1 kez) durgun
            # bir önizleme olarak güncelleniyor, sadece büyük/seçili kamera
            # tam hızda akıyor.
            if simdi_render - getattr(self, '_son_kucuk_render_zamani', 0.0) >= 1.0:
                self._son_kucuk_render_zamani = simdi_render
                pix1 = self.resmi_yuvarla(goruntu_sozlugu.get(1), 48, silah_etiketi)
                pix2 = self.resmi_yuvarla(goruntu_sozlugu.get(2), 48, self.kamera_yazi_on)
                pix3 = self.resmi_yuvarla(goruntu_sozlugu.get(3), 48, self.kamera_yazi_arka)
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
        # DÜZELTME: editingFinished odak KAYBINDA da tetikleniyor - kutuda
        # onceden ONAYLANMIS bir deger dururken (basarili onay sonrasi kutu
        # temizlenmiyor) baska bir yere (orn. kapatma butonuna) tiklamak
        # odagi kacırıp AYNI degeri tekrar tekrar onaya sokuyordu - "kapatma
        # butonuna basinca pwm onay ekrani her seferinde cikiyor" olarak
        # canli bildirildi. Deger degismediyse tekrar sormaya gerek yok.
        if metin == getattr(self, '_pwm_son_uygulanan_metin', None):
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
                self._pwm_son_uygulanan_metin = kutu.text()
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
        # DÜZELTME: butona tıklayınca kalan odak (focus) ana pencereye
        # geri çekilmezse keyPressEvent tuşları hiç almıyordu - klavyeyi
        # aktif edip WASD'a basınca araç sürülemiyordu (canlı bildirildi).
        self.setFocus()
        # YENİ DÜZELTME (2026-08-31): pushButton_klavye Ayarlar sayfasında
        # (index 4) duruyor, ama keyPressEvent SADECE [0,1,2] sayfalarında
        # tuş kabul ediyor (kasıtlı - ayar kutularına yazarken yanlışlıkla
        # araç sürülmesin diye). Kullanıcı Ayarlar sayfasındayken klavyeyi
        # aktif edip hemen WASD'a basınca "aktif ettim ama süremiyorum"
        # oluyordu - klavye aktif olunca otomatik ana ekrana dön.
        # DÜZELTME (2026-09-01, ikinci kez canlı bildirildi - "manuele
        # basınca / klavyeyi aktif edince hâlâ atıyor"): sayfa bazlı korumalı
        # otomatik yönlendirme (currentIndex() not in (0,1,2)) kod
        # incelemesinde doğru görünse de kullanıcı hâlâ istenmeyen bir
        # sayfa değişikliği bildirdi. Kesin çözüm için otomatik sayfa
        # DEĞİŞTİRME tamamen kaldırıldı - artık klavye hangi sayfadan
        # aktif edilirse edilsin O SAYFADA kalınır, HİÇBİR ZAMAN başka bir
        # sayfaya atlanmaz. Bedeli: Ayarlar sayfasındayken (index 4) klavye
        # aktif edilip hemen WASD'a basılırsa hâlâ tepki vermez (keyPressEvent
        # kasıtlı olarak sadece [0,1,2] sayfalarında tuş kabul ediyor) - ama
        # bu artık sessiz/beklenen bir durum, kullanıcı manuel olarak Ana/
        # Kamera/Navigasyon sayfasına geçtiğinde klavye zaten aktif olacak.
        pass

    # =====================================================================
    # YER İSTASYONU FİZİKSEL KONTROL PANELİ  (kontrol_paneli_sistemi.py)
    # Sol joystick=araç sür · Sağ joystick=silah · Pot=PWM üst sınırı ·
    # Sol toggle=ACİL STOP · Sağ toggle=silah ateş · Sol buton=Manuel/Otonom ·
    # Sağ buton=farlar. Açılışta otomatik başlar (bkz. _kontrol_paneli_baslat).
    # =====================================================================
    def _kontrol_paneli_baslat(self):
        # DUZELTME (2026-09-06, kullanıcı: "arduinodan çektiğimiz verileri
        # arka tarafta bir panelden gönderelim, arayüz üzerinde kasma
        # oluyor lidar ekranında") - ARTIK PANEL BU PROCESS'TE OKUNMUYOR.
        #
        # Kök neden (nihayet): KontrolPaneliThread bu process'in İÇİNDE bir
        # QThread'di - yani arayüzle AYNI GIL'i paylaşıyordu. Taktik Radar
        # (lidar) ekranı açıkken GUI thread her karede CPU-yoğun çizim
        # yaparken GIL'i tutuyor; joystick thread'i veriyi okusa bile
        # yayınlamak için GIL'i beklemek zorunda kalıyordu. Daha önce
        # denenenler (turret BEST_EFFORT QoS, Qt.DirectConnection,
        # paintEvent'in 29.7ms->11ms optimizasyonu) marjı genişletti ama
        # GIL rekabetini ORTADAN KALDIRAMAZ - çünkü sorun Qt kuyruğu
        # değil, yorumlayıcı kilidiydi.
        #
        # Artık panel AYRI BİR PROCESS'te (kontrol_paneli_node.py, ayrı GIL)
        # okunuyor ve joystick/acil-stop/silah komutlarını DOĞRUDAN ROS2'ye
        # yayınlıyor. Arayüz ne kadar meşgul olursa olsun (hatta donsa
        # bile) bu komutlar kesintisiz gider - bu aynı zamanda bir GÜVENLİK
        # İYİLEŞTİRMESİDİR: ACİL STOP artık arayüzün sağlığına bağlı değil.
        # Bu process sadece /panel_durumu'na abone olup GÖSTERGELERİ
        # günceller (gecikmeye duyarsız).
        #
        # KRİTİK: iki process AYNI seri portu açamaz - bu yüzden burada
        # KontrolPaneliThread ARTIK BAŞLATILMIYOR.
        self.kontrol_paneli_motoru = None
        # ÇİFT BAŞLATMA KORUMASI (2026-09-06): bu metot birden fazla
        # yerden çağrılıyor (__init__, telemetri kurulumu, joystick
        # aç/kapa butonu). İKİ node AYNI seri portu açamaz - zaten
        # çalışan sağlıklı bir süreç varsa yenisi başlatılmaz.
        _mevcut = getattr(self, '_panel_node_süreci', None)
        if _mevcut is not None and _mevcut.poll() is None:
            return
        self._panel_node_süreci = None
        # ORPHAN TEMİZLİĞİ (2026-09-06): arayüz çöküp süpervizör tarafından
        # yeniden başlatıldığında, ÖNCEKİ main.py'nin başlattığı node
        # sahipsiz (orphan) olarak yaşamaya devam eder. Node'un kendi
        # tekil-çalışma kilidi ikinci bir örneğin açılmasını engeller, ama
        # o zaman da bu process'in node'u HİÇ başlayamaz ve bekçi boşuna
        # denemeye devam eder. Bu yüzden başlamadan önce eski örnekler
        # temizlenir - tek sahiplik garanti edilir.
        # NOT (2026-09-06): burada eski/orphan node'ları temizlemek için
        # pkill DENENDİ ve İKİ KEZ geri alındı:
        #   1) subprocess.run(timeout=5)+sleep -> burası GUI thread'i
        #      (__init__) olduğu için arayüz AÇILIŞTA dondu ("force quit").
        #   2) subprocess.Popen (non-blocking) -> bu sefer pkill, hemen
        #      ardından başlatılan YENİ node ile de eşleşip onu öldürme
        #      yarışı yarattı.
        # Temizliğe zaten GEREK YOK: node kendi tekil-çalışma kilidini
        # tutuyor (bkz. kontrol_paneli_node.py::_tekil_calisma_kilidi) -
        # orphan bir node varsa O çalışmaya devam eder (joystick çalışır),
        # ikinci örnek sessizce çıkar. Bekçi de bunu pgrep ile görüp
        # boşuna yeniden başlatmaya çalışmaz (bkz. _panel_bekci_kontrol).
        try:
            # Node'un çıktısı bir log dosyasına yazılır - DEVNULL YAPMA:
            # ayrı process olduğu için sorun çıktığında (Arduino bağlanmadı,
            # port meşgul vb.) tek teşhis kaynağı burasıdır.
            _panel_log = open("/tmp/tufan_kontrol_paneli_node.log", "a")
            _panel_log.write(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} yeniden başlatıldı ---\n")
            _panel_log.flush()
            self._panel_node_süreci = subprocess.Popen(
                [sys.executable, "-u",
                 os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "kontrol_paneli_node.py")],
                stdout=_panel_log, stderr=subprocess.STDOUT)
            self.log_yaz("🎮 Kontrol paneli AYRI PROCESS olarak başlatıldı "
                         "(joystick artık arayüz yükünden bağımsız).")
        except Exception as e:
            self.log_yaz(f"⚠️ Kontrol paneli süreci başlatılamadı: {e}")

    def _panel_surecini_durdur(self):
        """Ayrı process olarak çalışan kontrol paneli node'unu sonlandırır.
        SIGTERM gönderilir (node bunu yakalayıp seri portu temiz kapatır),
        kısa bir süre beklenir, gerekirse zorla kapatılır."""
        sur = getattr(self, '_panel_node_süreci', None)
        if sur is None:
            return
        try:
            if sur.poll() is None:
                sur.terminate()
                try:
                    sur.wait(timeout=3)
                except Exception:
                    sur.kill()
        except Exception:
            pass
        self._panel_node_süreci = None

    def _panel_komut_gonder(self, komut):
        """PANEL TESTİ butonları -> ayrı process'teki panel node'u
        (bkz. kontrol_paneli_node.py::_komut_cb). Panel artık bu process'te
        olmadığı için metot çağrısı yerine ROS2 topic'i kullanılıyor."""
        hy = getattr(self, 'harita_yoneticisi', None)
        if hy is None or getattr(hy, 'ros_motoru', None) is None:
            return False
        return hy.ros_motoru.panel_komut_gonder(komut)

    def _panel_durumu_geldi(self, veri):
        """Ayrı process'teki kontrol paneli node'undan gelen gösterge
        güncellemeleri (bkz. kontrol_paneli_node.py::durum_yayinla).
        Joystick/acil-stop/silah komutları BURADAN GEÇMEZ - onları node
        doğrudan ROS2'ye yayınlıyor; burada sadece ekrandaki göstergeler
        güncelleniyor, bu yüzden gecikmesi önemsiz."""
        tip = veri.get('tip')
        if tip == 'baglanti':
            self._panel_baglanti_degisti(bool(veri.get('bagli')))
        elif tip == 'acil_stop_manuel':
            # ACİL STOP -> MANUEL (2026-09-06, kullanıcı isteği). Komutun
            # kendisini panel node DOĞRUDAN araca gönderdi; burada arayüzün
            # KENDİ durumu da MANUEL yapılıyor. Bu şart: arayüz
            # /surus_modu'nu HER TURDA tekrar yayınlıyor
            # (telemetri_sistemi.py::surekli_yayin_dongusu), kendi modunu
            # değiştirmezse bir sonraki turda OTONOM'u geri yazar ve acil
            # stop'un mod değişimi ANINDA geri alınırdı.
            mod = getattr(getattr(self, 'telemetri_motoru', None),
                          'arac_modu', 'MANUEL')
            if mod != "MANUEL":
                self.manuel_sec()
                self.log_yaz("🛑 ACİL STOP: sürüş modu OTONOM → MANUEL'e alındı, "
                             "otonom hedef üretimi durduruldu.")
            self.acil_durdurma()
        elif tip == 'fren':
            # FREN (2026-09-06): fiziksel sol buton (D9) artik surus modunu
            # DEGIL freni degistiriyor. Komutun kendisi ayri process'teki
            # kontrol_paneli_node'dan DOGRUDAN araca gidiyor (/fren_komut);
            # buraya sadece operatore gosterilecek bilgi geliyor.
            acik = bool(veri.get('acik'))
            self.log_yaz(
                f"🅿️ FİZİKSEL BUTON: FREN {'AÇILIYOR' if acik else 'KAPANIYOR'} "
                f"(araçta motor 3 sn çalışacak)")
        elif tip == 'sol_buton':
            # Geriye donuk uyumluluk: eski kontrol_paneli_node calisiyorsa
            # (yeni surum yuklenmeden once baslatilmissa) hala bu tip gelir.
            # Sessizce yok sayiliyor - mod degistirme artik EKRAN
            # butonlariyla (MANUEL/OTONOM) yapiliyor.
            pass
        elif tip == 'sag_buton':
            self._farlar_toggle()
        elif tip == 'pot':
            self._panel_pot_geldi(int(veri.get('pwm', 0)))
        elif tip == 'yer_batarya':
            self.yer_batarya_renklendir(str(veri.get('metin', '')))
        elif tip == 'log':
            self.log_yaz(str(veri.get('mesaj', '')))
        elif tip == 'ham':
            # PANEL TESTİ dialogu açıkken canlı ham veri (bkz.
            # _panel_testi_ac içindeki guncelle fonksiyonu).
            gnc = getattr(self, '_panel_ham_guncelle', None)
            if gnc is not None:
                try:
                    gnc(veri.get('veri', {}))
                except Exception:
                    pass

    def _panel_surus_geldi(self, sol_oran, sag_oran):
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.joystick_pwm_gonder(sol_oran, sag_oran)

    def _panel_turret_geldi(self, x, y):
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.joystick_turret_gonder(x, y)

    def _panel_pot_geldi(self, deger):
        # Potansiyometre (A5) -> PWM üst sınırı. CANLI, onaysız (histerezis +
        # throttle thread tarafında). Ayarlar'daki kutu + ekran altı OSD
        # göstergesi güncellenir. NOT: sessiz=True -> her küçük oynamada log
        # satırı basılmasın.
        if hasattr(self, 'telemetri_motoru'):
            uygulanan = self.telemetri_motoru.pwm_ust_sinirini_ayarla(deger, sessiz=True)
        else:
            uygulanan = float(deger)
        kutu = self.ui.lineEdit_kullaniciAdi_2
        kutu.setText(f"{uygulanan:.0f}")
        kutu.setStyleSheet(PWM_LIMIT_AKTIF)
        self._pwm_son_uygulanan_metin = kutu.text()
        self._pwm_osd_goster(uygulanan)

    def _panel_acil_stop(self, aktif):
        # Sol toggle switch. GND'ye alınca ACİL STOP; bırakınca OTOMATİK DEVAM
        # (kullanıcı kararı: switch tek yetkili). Thread sadece kenar yayınlar.
        if aktif:
            self.log_yaz("🛑 FİZİKSEL ANAHTAR: Sol toggle GND'ye alındı → ACİL STOP")
            self.acil_durdurma()
        else:
            self.log_yaz("✅ FİZİKSEL ANAHTAR: Sol toggle bırakıldı → DEVAM (otomatik)")
            if hasattr(self, 'telemetri_motoru'):
                self.telemetri_motoru.hareket_emri_gonder("DEVAM_CMD")
            self._arac_kilitli_mi = False
            if hasattr(self.ui, 'pushButton_devamEt'):
                self.ui.pushButton_devamEt.setVisible(False)
            self.ui.terminal_ekrani.append(
                "<br><span style='color:#00FF7B;'><b>✅ DEVAM (fiziksel anahtar): "
                "acil durdurma kilidi açıldı, araç tekrar komut kabul ediyor.</b></span><br>"
            )

    def _silah_fazi_degisti(self, aktif):
        """9. tabela silah fazı (kullanıcı isteği, 2026-09-06:
        "9 numarayı okuyunca silah otonoma geçsin, araç manuele").

        Araç tarafı /surus_modu=MANUEL'i ZATEN doğrudan yayınlıyor, ama
        arayüz de her turda kendi modunu yayınladığı için (bkz.
        telemetri_sistemi.surekli_yayin_dongusu) burada arayüzün KENDİ
        durumu da değişmezse bir sonraki turda OTONOM geri yazılır ve
        değişiklik anında geri alınır - acil stop → MANUEL'de yaşanan
        çakışmanın aynısı.

        Faz bitince, faz BAŞLAMADAN ÖNCEKİ mod geri yüklenir: araç
        OTONOM'da seyrederken silah fazına girdiyse görevine OTONOM
        devam eder; zaten MANUEL'deyse MANUEL kalır (operatörün kararı
        kendiliğinden değiştirilmez)."""
        mod = getattr(getattr(self, 'telemetri_motoru', None), 'arac_modu', 'MANUEL')
        if aktif:
            self._silah_fazi_onceki_mod = mod
            if mod != "MANUEL":
                # Bu MANUEL geçişi FAZIN KENDİSİ tarafından yapılıyor -
                # operatör isteği değil, o yüzden iptal tetiklenmemeli.
                self._silah_fazi_mod_degisimi = True
                try:
                    self.manuel_sec()
                finally:
                    self._silah_fazi_mod_degisimi = False
            self.log_yaz("🔫 SİLAH FAZI (9. tabela): SİLAH OTONOM, ARAÇ MANUEL'e alındı.")
        else:
            onceki = getattr(self, '_silah_fazi_onceki_mod', 'MANUEL')
            if onceki == "OTONOM" and mod != "OTONOM":
                self.otonom_sec()
                self.log_yaz("🏁 Silah fazı bitti: araç OTONOM'a geri döndü.")
            else:
                self.log_yaz("🏁 Silah fazı bitti (araç MANUEL kalıyor).")

    def _panel_mod_toggle(self):
        # ARTIK FİZİKSEL BUTONA BAĞLI DEĞİL (2026-09-06, kullanıcı isteği:
        # sol buton FREN oldu, bkz. kontrol_paneli_node.py::fren_toggle).
        # Metot, ekrandan/başka bir yerden çağrılabilsin diye duruyor.
        # Her basışta MANUEL <-> OTONOM.
        mod = getattr(getattr(self, 'telemetri_motoru', None), 'arac_modu', 'MANUEL')
        if mod == "MANUEL":
            self.otonom_sec()
        else:
            self.manuel_sec()
        self.log_yaz("🎮 FİZİKSEL BUTON: sürüş modu değiştirildi.")

    def _panel_baglanti_degisti(self, bagli):
        # *** KENAR TETIKLEME - CANLI OLCUMDE BULUNAN HATA (2026-09-07) ***
        # kontrol_paneli_node bağlantı durumunu ~2 saniyede bir TEKRAR
        # yayınlıyor (arayüz geç başlarsa sürüş kaynağını öğrensin diye
        # 2026-09-06'da eklendi). Ama bu metot her tekrarda BAŞTAN
        # çalışıyordu ve içindeki silah_manuel_moduna_al() her seferinde
        # /silah_modu'na MANUEL yazıyordu. turret_node OTONOM'dan çıkınca
        # PID'i ve hedef takibini SIFIRLIYOR, lazeri kapatıyor - yani
        # silah otonom hedeflemedeyken 2 saniyede bir sıfırlanıyordu ve
        # HEDEFE HİÇ KİLİTLENEMİYORDU. Ölçüm: 10 saniyede /silah_modu'na
        # 67 OTONOM arasında 8 MANUEL karışıyordu.
        # Çözüm: periyodik tekrar KALSIN (amacına hizmet ediyor) ama
        # eylemler yalnızca durum GERÇEKTEN değiştiğinde çalışsın.
        # İlk mesajda _panel_bagli_son None olduğu için değişim sayılır,
        # yani geç başlayan arayüz yine doğru kaynağı öğrenir.
        if bagli == getattr(self, '_panel_bagli_son', None):
            return
        self._panel_bagli_son = bagli
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        if bagli:
            self.log_yaz("🎮 Kontrol paneli Arduino bağlandı.")
            if hasattr(self, 'telemetri_motoru'):
                self.telemetri_motoru.surus_kaynagi = "JOYSTICK"
                self.telemetri_motoru.silah_manuel_moduna_al()
            if getattr(self, 'joystick_bagli', True):
                self.ui.pushButton_joystick.setStyleSheet(AYAR_AKTIF)
                self.ui.pushButton_joystick.setText(dil["btn_bagli"])
        else:
            self.log_yaz("⚠️ Kontrol paneli Arduino bağlantısı yok/koptu — klavye sürüşü aktif.")
            # Arduino çıkarılırsa sürücü klavyeyle sürebilsin diye kaynağı geri al.
            if hasattr(self, 'telemetri_motoru') and self.telemetri_motoru.surus_kaynagi == "JOYSTICK":
                self.telemetri_motoru.surus_kaynagi = "KLAVYE"
                if not getattr(self, 'klavye_aktif', False):
                    self.ayar_klavye_tetikle()
            if getattr(self, 'joystick_bagli', True):
                self.ui.pushButton_joystick.setStyleSheet(AYAR_PASIF)
                self.ui.pushButton_joystick.setText(dil["btn_baglantiyok"])

    def ayar_joystick_tetikle(self):
        # Ayarlar'daki "Joystick" butonu artık kontrol panelini YAZILIMSAL
        # olarak aç/kapatır (donanım açılışta otomatik başlar).
        dil = CEVIRILER.get(self.aktif_dil, CEVIRILER["Türkçe"])
        self.joystick_bagli = not getattr(self, 'joystick_bagli', True)
        if self.joystick_bagli:
            self.ui.pushButton_joystick.setStyleSheet(AYAR_AKTIF)
            self.ui.pushButton_joystick.setText(dil["btn_bagli"])
            self.log_yaz("Donanım: Kontrol paneli portları taranıyor...")
            # Panel artık AYRI PROCESS (bkz. _kontrol_paneli_baslat) -
            # thread yerine sürecin yaşayıp yaşamadığına bakılıyor.
            _sur = getattr(self, '_panel_node_süreci', None)
            if _sur is None or _sur.poll() is not None:
                self._kontrol_paneli_baslat()
        else:
            self.ui.pushButton_joystick.setStyleSheet(AYAR_PASIF)
            self.ui.pushButton_joystick.setText(dil["btn_baglantiyok"])
            self.log_yaz("Donanım: Kontrol paneli YAZILIMSAL OLARAK KESİLDİ.")
            if hasattr(self, 'telemetri_motoru'):
                self.telemetri_motoru.surus_kaynagi = "KLAVYE"
            self._panel_surecini_durdur()
            # Kaynak KLAVYE'ye döndüğü an klavyeyi de otomatik aç (yoksa
            # "joystick kapatıp klavye açtığımda süremiyorum" - canlı bildirildi).
            if not getattr(self, 'klavye_aktif', False):
                self.ayar_klavye_tetikle()
        self.setFocus()

    # --- PWM üst sınırı ekran-altı göstergesi (ses OSD'si gibi belir/sön) ---
    def _pwm_osd_olustur(self):
        osd = QLabel(self.ui.centralwidget)
        osd.setObjectName("pwmOsd")
        osd.setAlignment(Qt.AlignCenter)
        osd.setStyleSheet(
            "QLabel { background-color: rgba(13, 20, 28, 235); color: #E6F7FF; "
            "border: 1px solid #00E5FF; border-radius: 14px; padding: 12px 26px; "
            "font-size: 18px; font-weight: bold; }"
        )
        self._pwm_osd_efekt = QGraphicsOpacityEffect(osd)
        osd.setGraphicsEffect(self._pwm_osd_efekt)
        self._pwm_osd_anim = QPropertyAnimation(self._pwm_osd_efekt, b"opacity", self)
        self._pwm_osd_gizle_zamanlayici = QTimer(self)
        self._pwm_osd_gizle_zamanlayici.setSingleShot(True)
        self._pwm_osd_gizle_zamanlayici.timeout.connect(self._pwm_osd_solmaya_basla)
        osd.hide()
        self.ui.pwmOsd = osd

    def _pwm_osd_goster(self, deger):
        if not hasattr(self.ui, 'pwmOsd'):
            self._pwm_osd_olustur()
        osd = self.ui.pwmOsd
        oran = max(0.0, min(1.0, (float(deger) - 85.0) / (255.0 - 85.0)))
        dolu = int(round(oran * 22))
        cubuk = "█" * dolu + "░" * (22 - dolu)
        osd.setText(f"PWM ÜST SINIRI   ·   {float(deger):.0f}\n{cubuk}\nmin 85   —   max 255")
        osd.adjustSize()
        # DÜZELTME (2026-09-04, kullanici: "gösterge ekranın altındaki
        # gözükmüyor") - BUG: osd, centralwidget'in ÇOCUĞU (bkz.
        # _pwm_osd_olustur) - move() PARENT'A GÖRE koordinat alır. Burada
        # yanlışlıkla QApplication.primaryScreen().geometry() (TÜM masaüstü
        # ekranının boyutu, GLOBAL koordinat) kullanılıyordu - pencere/
        # centralwidget tam ekranla piksel piksel eşleşmiyorsa (pencere
        # kenarlıkları, onboard klavye açılışında tam ekrandan çıkma vb.,
        # bkz. proje notu) hesaplanan Y konumu centralwidget'in GERÇEK
        # sınırlarının DIŞINA taşıyor - Qt görünmez alanı kırpıyor, OSD
        # hiç görünmüyordu. Artık GERÇEK ebeveynin (centralwidget) kendi
        # boyutuna göre konumlandırılıyor.
        ust_widget = osd.parentWidget() or self.ui.centralwidget
        osd.move((ust_widget.width() - osd.width()) // 2, ust_widget.height() - osd.height() - 90)
        self._pwm_osd_anim.stop()
        self._pwm_osd_efekt.setOpacity(1.0)
        osd.show()
        osd.raise_()
        self._pwm_osd_gizle_zamanlayici.start(1400)

    def _pwm_osd_solmaya_basla(self):
        osd = self.ui.pwmOsd
        self._pwm_osd_anim.stop()
        self._pwm_osd_anim.setDuration(600)
        self._pwm_osd_anim.setStartValue(1.0)
        self._pwm_osd_anim.setEndValue(0.0)
        try:
            self._pwm_osd_anim.finished.disconnect()
        except TypeError:
            pass
        self._pwm_osd_anim.finished.connect(osd.hide)
        self._pwm_osd_anim.start()

    def _farlar_toggle(self):
        # Hem "F" tuşu hem kontrol panelindeki sağ buton buradan geçer -
        # /farlar (Bool) topic'ine AÇIK/KAPALI yayınlanır. Farlar bir sürüş
        # komutu değil yardımcı ekipman: OTONOM modda da çalışır.
        self.farlar_acik = not getattr(self, 'farlar_acik', False)
        if hasattr(self, 'telemetri_motoru'):
            self.telemetri_motoru.farlar_ayarla(self.farlar_acik)

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
        # ESC: rota/hedef oku çizerken (taktik radarda sürükleme) iptal eder
        # - klavye_aktif/mod/sayfa şartlarından BAĞIMSIZ çalışır, çünkü bu
        # sürüş komutu değil, harita etkileşimini iptal etme işlemi.
        if event.key() == Qt.Key_Escape and hasattr(self, 'harita_yoneticisi'):
            self.harita_yoneticisi.taktik_radar.hedef_cizimini_iptal_et()
            return
        if not self.klavye_aktif: return
        if self.ui.stackedWidget.currentIndex() not in [0, 1, 2]: return

        # FARLAR (2026-09-01, kullanıcı isteği): "F" tuşuna basınca AÇIK/
        # KAPALI arasında geçiş yapıp /farlar topic'ine (Bool) yayınlanıyor.
        # KASITLI OLARAK aşağıdaki "sadece MANUEL modda" şartından ÖNCE
        # işleniyor - farlar bir sürüş komutu değil yardımcı ekipman,
        # OTONOM modda da (ör. gece/tünel/rampa) açılabilmeli.
        # event.isAutoRepeat() koruması: tuşu basılı tutunca OS'un ürettiği
        # tekrarlı keyPress'ler yüzünden AÇIK/KAPALI arasında hızlıca
        # ÇIRPINMASIN diye - sadece gerçek/tek basışta geçiş yapılır.
        if event.key() == Qt.Key_F and not event.isAutoRepeat():
            self._farlar_toggle()
            return

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

    def closeEvent(self, event):
        # 2026-09-04, kullanıcı: "arayüzün bir anda çökmemesi lazım çok
        # nadiren de olsa oluyor" - KÖK NEDEN BULUNDU: pencere kapanırken
        # (X butonu, "Sistemi Kapat", Alt+F4) hiçbir closeEvent YOKTU -
        # QThread tabanlı worker'lar (kamera/telemetri/ntrip/kontrol
        # paneli/harita ROS motoru) HALA ÇALIŞIRKEN Qt nesneleri direkt
        # yok ediliyordu. Qt bunu "QThread: Destroyed while thread is
        # still running" FATAL hatasıyla (terminate called without an
        # active exception) SESSİZCE çökerek cezalandırıyor - canlı
        # app_log'da BİREBİR bu iz bulundu. Artık hepsi kapanmadan ÖNCE,
        # BLOKE EDEREK (quit()+wait(), bkz. ilgili dosyalardaki durdur()/
        # stop() metotları) düzgünce durduruluyor.
        if getattr(self, '_kapatma_tamamlandi', False):
            event.accept()
            return
        # Kontrol paneli AYRI PROCESS (2026-09-06) - EN ÖNCE sonlandırılır
        # ki ROS2 context'i kapanırken hâlâ komut yayınlamaya çalışmasın.
        self._panel_surecini_durdur()
        for ad, yontem in (
            ('kontrol_paneli_motoru', 'durdur'),
            ('kamera_motoru', 'stop'),
            ('telemetri_motoru', 'durdur'),
            ('ntrip_motoru', 'durdur'),
        ):
            nesne = getattr(self, ad, None)
            if nesne is not None:
                try:
                    getattr(nesne, yontem)()
                except Exception:
                    pass
        if hasattr(self, 'harita_yoneticisi'):
            try:
                self.harita_yoneticisi.kapat()
            except Exception:
                pass
        self._kapatma_tamamlandi = True
        # DUZELTME (2026-09-06): "-u" bayrağı kaldırılıp stdout tekrar
        # tamponlu yapıldığı için (bkz. calistir.sh notu), TEMİZ kapanışta
        # tampondaki son satırların kaybolmaması için burada MANUEL flush -
        # normal çalışma sırasında GUI thread'i BLOKE ETME riski yok
        # (sadece kapanış anında, TEK SEFERLİK).
        try:
            sys.stdout.flush()
        except Exception:
            pass
        event.accept()

_ANA_PENCERE_REF = None


def _kirilmaz_hata_yakalayici(exc_type, exc_value, exc_tb):
    """2026-09-04, kullanıcı isteği: "arayüzün bir anda çökmemesi lazım
    çok nadiren de olsa oluyor" - PyQt5'in VARSAYILAN davranışı: bir slot/
    callback içinde YAKALANMAMIŞ bir Python hatası olursa (ör. ROS
    mesajından beklenmeyen bir alan, seri porttan bozuk bir satır, nadir
    bir race condition), traceback stderr'e yazılıp UYGULAMA SESSİZCE
    ÇÖKER (event loop C++ tarafında abort ediyor) - hiçbir hata penceresi/
    log satırı görülmeden arayüz anında kapanır. Kullanıcının tarif ettiği
    "bir anda çökme" tam olarak bu.

    Artık TÜM yakalanmamış hatalar burada tutuluyor: diske (crash_log.txt)
    yazılıyor ve (ana pencere zaten oluşmuşsa) log kutusuna basılıyor, ama
    UYGULAMA KAPANMIYOR - araç/harita/kontrol çalışmaya devam ediyor. Bir
    slot ortasında yarım kalan durum riski var ama bu, canlı bir yarışma
    yer istasyonu için "sessizce kapanmak"tan HER ZAMAN daha güvenli.
    """
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_tb)
        return
    metin = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
    zaman = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        crash_dosya = os.path.join(os.path.dirname(os.path.abspath(__file__)), "crash_log.txt")
        with open(crash_dosya, "a") as f:
            f.write(f"\n--- {zaman} ---\n{metin}\n")
    except Exception:
        pass
    sys.stderr.write(f"\n[YAKALANMAMIŞ HATA - UYGULAMA AYAKTA KALDI, bkz. crash_log.txt]\n{metin}\n")
    try:
        if _ANA_PENCERE_REF is not None:
            _ANA_PENCERE_REF.log_yaz(f"⚠️ Beklenmeyen hata yakalandı (uygulama çalışmaya devam ediyor): {exc_value}")
    except Exception:
        pass


def _gui_takilma_izleyici():
    """GEÇİCİ TEŞHİS (2026-09-04, "kesin sebebi bul" - kullanıcı: "hala çok
    kasma var yaklaşık 1.5sn civarı") - önceki ölçümler (resmi_yuvarla,
    setPixmap) TEK TEK hızlı çıktı (~12ms, ~0.1ms) ama video slot'unun
    GUI thread'e ULAŞMA aralığı arada 200-500ms'ye sıçrıyordu - yani sorun
    video kodunun İÇİNDE değil, GUI thread'in ARADA BAŞKA BİR YERDE
    (muhtemelen tek bir C-uzantısı çağrısında, ör. yavaş bir QPainter
    işlemi, subprocess/ağ çağrısı, ya da GIL'i uzun süre tutan bir işlem)
    TIKANIP KALMASIYDI. Bu, GUI thread'in KENDİSİ ile ölçülemez (tıkanan
    thread kendi ölçüm kodunu da çalıştıramaz) - AYRI bir arka plan
    thread'i, sys._current_frames() ile GUI thread'in o anki Python
    çağrı yığınını DIŞARIDAN periyodik örnekliyor. Yığın ÜST ÜSTE aynı
    satırda kalırsa (thread ilerlemiyor demektir), o satırı ve tam yığını
    konsola basıyor - "kesin sebep" burada görünecek. Sorun bulununca bu
    fonksiyon (ve çağrısı) kaldırılacak.
    """
    ana_id = threading.main_thread().ident
    son_imza = None
    tekrar = 0
    while True:
        time.sleep(0.03)
        try:
            kareler = sys._current_frames()
        except Exception:
            continue
        f = kareler.get(ana_id)
        if f is None:
            continue
        yigin = traceback.extract_stack(f)
        imza = (yigin[-1].filename, yigin[-1].lineno) if yigin else None
        if imza == son_imza:
            tekrar += 1
        else:
            if tekrar >= 5:  # ~150ms+ AYNI satırda takılı kalmış
                gecen_ms = tekrar * 30
                ozet = "".join(traceback.format_list(yigin[-8:]))
                print(f"\n[STALL] GUI thread ~{gecen_ms}ms boyunca aynı "
                      f"satırda tıkandı:\n{ozet}")
            tekrar = 0
            son_imza = imza


if __name__ == "__main__":
    # DÜZELTME (2026-09-05, kullanıcı: "yine çöktü, çöktüğü zaman sana
    # raporlasın neden çöktüğünü") - sys.excepthook SADECE Python
    # seviyesindeki hataları yakalar; GERÇEK bir native çökme (segfault/
    # SIGABRT - ör. Qt/C++ tarafında bir hata) hiçbir Python hatası
    # ÜRETMEZ, hiçbir şey loglamadan aniden ölür - bu oturumda defalarca
    # görülen "hiçbir iz yok" çökmelerinin muhtemel sınıfı bu.
    # `faulthandler` tam bunun için var: sudo/ptrace GEREKTİRMEDEN,
    # SIGSEGV/SIGABRT/SIGBUS/SIGILL/SIGFPE alındığı anda TÜM thread'lerin
    # o anki Python çağrı yığınını otomatik olarak dosyaya yazar - bir
    # sonraki "neden çöktü" sorusuna kesin bir cevap verir.
    import faulthandler
    _crash_dosya = os.path.join(os.path.dirname(os.path.abspath(__file__)), "crash_log.txt")
    _crash_f = open(_crash_dosya, "a")
    _crash_f.write(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} (faulthandler etkin) ---\n")
    _crash_f.flush()
    faulthandler.enable(file=_crash_f, all_threads=True)

    sys.excepthook = _kirilmaz_hata_yakalayici
    threading.Thread(target=_gui_takilma_izleyici, daemon=True).start()
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    pencere = TufanGCS()
    _ANA_PENCERE_REF = pencere
    pencere.show()
    sys.exit(app.exec_())