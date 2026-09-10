#!/usr/bin/env python3
"""Yer istasyonu fiziksel kontrol panelini AYRI BİR PROCESS olarak ROS2'ye bağlar.

NEDEN AYRI PROCESS (2026-09-06, kullanıcı: "arduinodan çektiğimiz verileri
arka tarafta bir panelden gönderelim, arayüz üzerinde kasma oluyor lidar
ekranında"):
--------------------------------------------------------------------------
Eskiden KontrolPaneliThread, main.py'nin İÇİNDE bir QThread'di - yani
arayüzle AYNI Python process'i, dolayısıyla AYNI GIL (Global Interpreter
Lock). Taktik Radar (lidar) ekranı açıkken GUI thread her karede CPU-yoğun
çizim yapıyor ve bu sırada GIL'i TUTUYOR; joystick thread'i seri porttan
veriyi okusa bile ROS2'ye yayınlamak için GIL'i beklemek zorunda kalıyordu.
Bu yüzden joystick komutları lidar ekranında gözle görülür şekilde
gecikiyordu.

Daha önce denenen ve YETERSİZ kalan adımlar (hepsi hâlâ geçerli/yararlı,
ama kök nedeni çözmüyorlardı):
  - turret_cmd_pub'ın BEST_EFFORT QoS'a alınması (DDS tıkanmasını çözdü)
  - Qt.DirectConnection (Qt sinyal KUYRUĞUNU bypass etti - ama GIL
    rekabetini çözemez, çünkü sorun kuyruk değil, yorumlayıcı kilidiydi)
  - paintEvent optimizasyonları (29.7ms -> ~11ms; marjı genişletti ama
    GUI yoğun oldukça joystick yine sıraya giriyordu)

AYRI PROCESS = AYRI GIL. Bu node, arayüz ne kadar meşgul olursa olsun
(hatta arayüz tamamen donsa/kapansa bile) joystick + acil stop + silah
komutlarını kesintisiz yayınlamaya devam eder. Bu aynı zamanda bir
GÜVENLİK İYİLEŞTİRMESİDİR: ACİL STOP artık arayüzün sağlığına bağlı değil.

TASARIM:
--------------------------------------------------------------------------
Kod TEKRARI YOK: kontrol_paneli_sistemi.py'deki KontrolPaneliThread sınıfı
(tüm kalibrasyon, ölü bölge, oto-merkezleme, güvenlik ağı, debounce
mantığıyla birlikte) OLDUĞU GİBİ kullanılıyor. Tek fark: Qt sinyalleri
arayüz slot'larına değil, buradaki ROS2 yayıncılarına bağlanıyor.
QThread sinyalleri için bir Qt olay döngüsü gerektiğinden GUI'siz bir
QCoreApplication kullanılıyor (pencere açmaz, X11'e dokunmaz).

YAYINLADIKLARI (arayüzün telemetri_sistemi.py'de yaptığının AYNISI -
aynı topic, aynı QoS, aynı ölçekleme/işaret kuralları):
  /palet_hizlari      Float32MultiArray  BEST_EFFORT depth=1  (sürüş)
  /turret_manuel_cmd  Twist              BEST_EFFORT depth=1  (silah pan/tilt)
  /silah_ates_manuel  Bool                                    (ateş)
  /arac_komut         String                                  (ACİL STOP/DEVAM)
  /panel_durumu       String (JSON)      arayüzün göstergeleri için

ABONE:
  /surus_modu         String   - OTONOM'dayken sürüş komutu YAYINLANMAZ
                                 (arayüzdeki arac_modu kontrolünün aynısı)
"""

import fcntl
import json
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Bool, Float32MultiArray, Int32
from geometry_msgs.msg import Twist

from PyQt5.QtCore import QCoreApplication, QTimer

from kontrol_paneli_sistemi import (
    KontrolPaneliThread, PWM_ALT, PWM_UST, _kirp,
)

# Arayüz tarafındaki telemetri_sistemi.py ile AYNI değerler - sürüş
# komutu bu süre içinde tazelenmezse paletler frenlenir (ölü adam butonu).
GUVENLIK_ZAMAN_ASIMI_S = 0.8
# Lineer fren motoru 3 sn hareket eder; bu sure + pay dolmadan gelen
# buton basislari yok sayilir (bkz. fren_toggle'daki canli hata notu).
FREN_KILIT_S = 3.5
# Sürüş komutu yayın hızı (arayüzdeki surekli_yayin_dongusu ile aynı).
YAYIN_ARALIGI_S = 0.1

# --- MANUEL SILAH ADIM SAYISI (2026-09-09, kullanici istegi: "manuelde
# silah hizini ayarlamak icin arac hizini ayarlayan pot ile ayni yap, pot
# 0'dayken 2 adim atsin, pot 1023'te iken 100 adim atsin, ama aracin hiz
# ayarini bozma ayni kalsin").
#
# ONEMLI - ARAC HIZ HARITASI HIC DEGISMEDI: pot okuma/EMA/PWM haritasi
# (kontrol_paneli_sistemi.py, ham -> 85..255) OLDUGU GIBI DURUYOR. Adim
# sayisi o haritanin CIKTISINDAN geri cozuluyor:
#     pwm = PWM_ALT + (ham/1023) * (PWM_UST - PWM_ALT)
#  => ham/1023 = (pwm - PWM_ALT) / (PWM_UST - PWM_ALT)
# Yani ayni ham pot degerine BAGIMLI, ikinci bir okuma/yayin yolu ACMADAN
# (tek kaynak, tek throttle) adim sayisi turetiliyor.
SILAH_ADIM_MIN = 2      # pot 0    -> 2 adim   (hassas nisan)
SILAH_ADIM_MAX = 100    # pot 1023 -> 100 adim (hizli tarama)


class KontrolPaneliNode(Node):
    def __init__(self):
        super().__init__('kontrol_paneli_node')

        # BEST_EFFORT + depth=1: sürekli tekrarlanan kontrol sinyalleri için
        # (bkz. telemetri_sistemi.py'deki aynı gerekçeli not - RELIABLE'ın
        # ack/retransmit mekanizması bu ağda saniyelerce tıkanabiliyor).
        hizli_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.palet_pub = self.create_publisher(Float32MultiArray, '/palet_hizlari', hizli_qos)
        self.turret_pub = self.create_publisher(Twist, '/turret_manuel_cmd', hizli_qos)
        # Tek-seferlik/nadir güvenlik komutları RELIABLE kalır (garantili
        # teslim gerekir - bkz. DOKUMANTASYON.md'deki QoS notu).
        self.ates_pub = self.create_publisher(Bool, '/silah_ates_manuel', 10)
        self.komut_pub = self.create_publisher(String, '/arac_komut', 10)
        self.durum_pub = self.create_publisher(String, '/panel_durumu', 10)
        # FREN (2026-09-06, kullanici istegi: "yer istasyonundaki manuel
        # otonom gecisi butonu fiziksel olan onu fren olarak degistir, ona
        # bastigimizda freni acsin tekrar bastigimizda kapatsin, 1 kere
        # basinca 3 sn motoru calistirsin").
        # Sol buton (D9) ESKIDEN surus modunu degistiriyordu; artik freni
        # ac/kapa yapiyor. Mod degistirme ARAYUZDEKI ekran butonlariyla
        # (MANUEL/OTONOM) yapilmaya devam ediyor - kaybolan bir yetenek YOK.
        #
        # Komut ARAYUZDEN GECMEZ, bu ayri process'ten DOGRUDAN araca gider
        # (acil stop ile ayni desen) - arayuz donsa/kapansa bile fren
        # kumanda edilebilir. 3 saniyelik motor calisma suresini ARDUINO
        # sayar; buradan sadece HEDEF DURUM gonderilir.
        # MANUEL SILAH ADIM SAYISI (bkz. SILAH_ADIM_MIN/MAX): turret_node
        # MANUEL modda her heartbeat'te bu kadar adimlik "coklu adim" darbesi
        # gonderir - silahin manuel hareket hizi boylece surus potundan
        # ayarlanir. Surus PWM'i bu topic'ten ETKILENMEZ.
        self.silah_adim_pub = self.create_publisher(Int32, '/silah_adim_sayisi', 10)
        self.silah_adim_sayisi = SILAH_ADIM_MIN

        self.fren_pub = self.create_publisher(Bool, '/fren_komut', 10)
        self.fren_acik = False
        self._fren_son_komut = 0.0

        self.create_subscription(String, '/surus_modu', self._mod_cb, 10)
        # ACİL STOP -> MANUEL (2026-09-06, kullanici istegi: "araç otonomdayken
        # yer istasyonundaki acil stop çalışmıyor; acil stop switch'i manuele
        # geçirsin ve aracı durdursun").
        #
        # TESHIS: kilit (arduino_motor_kontrol._kilitli) ASLINDA CALISIYORDU -
        # canli olcumde kilitliyken araca giden PPM mesaji sayisi 0 cikti.
        # Eksik olan sey su: kilit SADECE motorlara giden komutu kesiyor,
        # OTONOMI ALTTA CALISMAYA DEVAM EDIYOR - Nav2/goal_manager hedef
        # uretmeyi surduruyor ve switch birakilir birakilmaz arac kaldigi
        # yerden DEVAM ediyor. Operator icin bu "acil stop calismiyor"
        # demek. Cozum: acil stop ayrica surus modunu MANUEL'e cekiyor.
        self.mod_pub = self.create_publisher(String, '/surus_modu', 10)
        # ARAYÜZ -> NODE KOMUT KANALI (2026-09-06, kullanıcı: "arayüzdeki
        # panel testini de entegre edelim, bazen joystick'i sıfırlamak
        # gerekiyor") - panel artık ayrı process olduğu için arayüz
        # KontrolPaneliThread'in metotlarını DOĞRUDAN çağıramaz; bu topic
        # o köprüyü kurar (PANEL TESTİ dialogundaki "Merkezi Sıfırla" ve
        # "Kaydet ve Uygula" butonları + ham veri akışının aç/kapası).
        self.create_subscription(String, '/panel_komut', self._komut_cb, 10)
        self.panel = None  # main() tarafından atanır (KontrolPaneliThread)

        # --- durum ---
        self.arac_modu = "MANUEL"
        self.pwm_ust_sinir = float(PWM_UST)
        self.anlik_sol_pwm = 0.0
        self.anlik_sag_pwm = 0.0
        self.son_komut_zamani = 0.0
        self.acil_stop_aktif = False
        self.ates_manuel_aktif = False
        self.panel_bagli = False

        self.get_logger().info(
            'kontrol_paneli_node aktif: Arduino AYRI PROCESS olarak okunuyor - '
            'joystick/acil-stop/silah komutları arayüzün GIL yükünden BAĞIMSIZ.')

    def _mod_cb(self, msg: String):
        yeni = msg.data.strip().upper()
        if yeni in ("MANUEL", "OTONOM"):
            self.arac_modu = yeni

    def _komut_cb(self, msg: String):
        """Arayüzden gelen panel komutları (bkz. /panel_komut aboneliği)."""
        if self.panel is None:
            return
        komut = msg.data.strip()
        try:
            if komut == "merkezi_sifirla":
                # Joystick merkezini yeniden kalibre et (joystickler
                # BIRAKILMIŞ varsayımıyla - arayüz kullanıcıyı uyarıyor).
                self.panel.merkezi_sifirla()
                self.get_logger().info('PANEL: joystick merkezi sıfırlama istendi')
            elif komut == "ayarlari_yeniden_yukle":
                self.panel.ayarlari_yeniden_yukle()
                self.get_logger().info('PANEL: kalibrasyon ayarları yeniden yükleniyor')
            elif komut == "ham_veri_ac":
                self.panel.ham_yayin_acik = True
            elif komut == "ham_veri_kapat":
                self.panel.ham_yayin_acik = False
        except Exception as e:
            self.get_logger().error(f'PANEL komutu işlenemedi ({komut}): {e}')

    def ham_veri_geldi(self, d):
        """PANEL TESTİ dialogu açıkken ham panel verisi (50Hz) - arayüze
        JSON olarak iletilir. Sadece dialog açıkken (ham_yayin_acik)
        üretildiği için normalde hiç maliyeti yoktur."""
        try:
            self.durum_yayinla({'tip': 'ham', 'veri': d})
        except Exception:
            pass

    # ---------------- panel sinyalleri -> ROS2 ----------------
    def surus_geldi(self, sol_oran, sag_oran):
        """Tank karışımı SONRASI oran (-1..1). PWM'e ölçeklenip saklanır;
        gerçek yayın periyodik döngüde (bkz. tick) yapılır - arayüzdeki
        joystick_pwm_gonder + surekli_yayin_dongusu ikilisiyle AYNI desen."""
        if self.arac_modu != "MANUEL" or self.acil_stop_aktif:
            return
        self.son_komut_zamani = time.time()
        self.anlik_sol_pwm = self._pwm_olcekle(sol_oran)
        self.anlik_sag_pwm = self._pwm_olcekle(sag_oran)

    def turret_geldi(self, x, y):
        # ACİL STOP SİLAHI DA DURDURUR (2026-09-09, sahada bulundu:
        # "acil stop kapalıyken silah hareket ediyor"). ESKİ tasarım silahı
        # KASITLI olarak acil stop'un dışında bırakıyordu ("acil stop araca
        # giden motor komutları içindir") - operatör için bu kabul edilemez.
        # Komutu YUTMAK yetmez, SIFIR yayınlamak gerekir: araç tarafındaki
        # heartbeat son komutu tekrarlıyor, susarsak taret son yönde
        # dönmeye devam ederdi.
        if self.acil_stop_aktif:
            x = y = 0.0
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        try:
            self.turret_pub.publish(msg)
        except Exception:
            pass  # kapanış yarışı - tek bir komutun düşmesi zararsız

    def ates_degisti(self, aktif):
        """Sağ toggle switch (D5) -> silah ateş.

        ARAÇ TARAFI HEARTBEAT BEKLİYOR: turret_node.py'deki
        _ATES_MANUEL_TIMEOUT_S = 0.5 - /silah_ates_manuel TEK SEFER
        gönderilirse lazer yarım saniye sonra KENDİLİĞİNDEN söner.
        (2026-09-06 sahada: "lazeri açıyorum fakat 1sn sonra sönüyor,
        switch açık olmasına rağmen".) Panel ayrı process'e taşınmadan
        önce bu tekrarı main.py'deki 50ms'lik _ates_zamanlayici yapıyordu;
        taşıma sırasında o zamanlayıcı devre dışı kaldığı için ateş
        kenar-tetiklemeli tek mesaja düşmüştü. Artık bayrak burada
        tutuluyor ve tick() her 100ms'de yeniden yayınlıyor (0.5sn'lik
        araç zaman aşımına 5 kat pay).
        """
        self.ates_manuel_aktif = bool(aktif)
        self._ates_yayinla()
        self.get_logger().info(
            f'SİLAH ATEŞ (fiziksel anahtar) -> {"AÇIK" if aktif else "kapalı"}')

    def _ates_yayinla(self):
        try:
            # ACİL STOP: anahtar hâlâ AÇIK konumda olsa bile lazer yanmaz.
            acik = bool(self.ates_manuel_aktif) and not self.acil_stop_aktif
            self.ates_pub.publish(Bool(data=acik))
        except Exception:
            pass

    def fren_toggle(self):
        """Fiziksel sol buton (D9). Her basista fren ACIK <-> KAPALI
        (lineer motor: bir yone 3 sn, sonraki basista ters yone 3 sn).

        *** CANLI TESTTE BULUNAN HATA (2026-09-06) ***: kullanici "fren
        calismiyor" dedi; panel node logunda TEK basisin 6 KEZ tetiklendigi
        goruldu (159-262 ms araliklarla - butonun kontak sicramasi
        kontrol_paneli_sistemi.py'deki ~40 ms'lik debounce penceresini
        asiyor). Her tetikleme durumu ters cevirdigi icin lineer motor
        surekli yon degistiriyor ve HICBIR YERE VARAMIYORDU.

        Cozum debounce'u uzatmak DEGIL (o, farlar/diger butonlarin
        davranisini da degistirirdi ve 260 ms'lik sicramayi yine
        yakalayamazdi) - dogru yer KOMUT seviyesi: lineer motor 3 saniye
        boyunca zaten mesgul, o sure bitmeden gelen basis FIZIKSEL olarak
        uygulanamaz, bu yuzden yok sayilir."""
        simdi = time.time()
        if (simdi - self._fren_son_komut) < FREN_KILIT_S:
            self.get_logger().warn(
                'FREN: onceki hareket surerken gelen basis YOK SAYILDI '
                f'(kalan ~{FREN_KILIT_S - (simdi - self._fren_son_komut):.1f} sn)')
            return
        self._fren_son_komut = simdi
        self.fren_acik = not self.fren_acik
        try:
            self.fren_pub.publish(Bool(data=bool(self.fren_acik)))
        except Exception as e:
            self.get_logger().error(f'FREN komutu yayinlanamadi: {e}')
            return
        self.get_logger().warn(
            f'FREN (fiziksel buton) -> {"ACILIYOR" if self.fren_acik else "KAPANIYOR"} '
            f'(araçta motor 3 sn calisacak)')
        self.durum_yayinla({'tip': 'fren', 'acik': bool(self.fren_acik)})

    def acil_stop_degisti(self, aktif):
        """Sol toggle. GND'ye alınınca ACİL STOP, bırakılınca DEVAM.
        GÜVENLİK: artık arayüzden BAĞIMSIZ - arayüz donsa/kapansa bile
        bu komut araca gider."""
        self.acil_stop_aktif = bool(aktif)
        msg = String()
        if aktif:
            msg.data = "EMERGENCY_STOP_CMD"
            self.anlik_sol_pwm = 0.0
            self.anlik_sag_pwm = 0.0
            # SİLAHI DA ANINDA SUSTUR (bkz. turret_geldi'deki not).
            self.ates_manuel_aktif = False
            self.turret_geldi(0.0, 0.0)
            self._ates_yayinla()
            self.get_logger().warn(
                'ACİL STOP (fiziksel anahtar) -> araca gönderildi + '
                'sürüş modu MANUEL\'e çekiliyor (otonomi durur)')
        else:
            msg.data = "DEVAM_CMD"
            # DIKKAT: birakinca OTONOM'a GERI DONULMEZ. Otonomiye devam
            # etmek operatorun BILINCLI karari olmali - arayuzden OTONOM
            # butonuna basmasi gerekir.
            self.get_logger().info(
                'DEVAM (fiziksel anahtar) -> araca gönderildi '
                '(mod MANUEL kalır, otonom için arayüzden seçin)')
        try:
            self.komut_pub.publish(msg)
            if aktif:
                self._manuele_cek()
            self._palet_yayinla()  # acil stopta paletleri ANINDA sıfırla
        except Exception:
            pass

    def _manuele_cek(self):
        """Sürüş modunu MANUEL'e çeker - HEM araca HEM arayüze.

        Araca DOĞRUDAN yayınlamak sart: arayuz donmus/kapanmis olsa bile
        surus_koprusu.py ve on_bosluk_nokta_atici.py MANUEL'i gorup
        otonom komut uretmeyi/iletmeyi keser. Arayuze de haber verilir,
        cunku arayuz /surus_modu'nu HER TURDA tekrar yayinliyor
        (telemetri_sistemi.py::surekli_yayin_dongusu) - kendi durumunu
        da MANUEL yapmazsa bir sonraki turda OTONOM'u geri yazardi."""
        try:
            self.mod_pub.publish(String(data='MANUEL'))
        except Exception:
            pass
        self.durum_yayinla({'tip': 'acil_stop_manuel'})

    def baglanti_degisti(self, bagli):
        """Arduino bağlandı/koptu. Durum ayrıca tick()'te ~2sn'de bir
        TEKRAR yayınlanır (arayüz sonradan başlasa bile öğrensin diye)."""
        self.panel_bagli = bool(bagli)
        self.durum_yayinla({'tip': 'baglanti', 'bagli': self.panel_bagli})

    def pot_geldi(self, pwm_degeri):
        self.pwm_ust_sinir = float(_kirp(pwm_degeri, PWM_ALT, PWM_UST))
        # AYNI pot degerinden manuel silah adim sayisi (arac hizina
        # DOKUNMADAN - bkz. SILAH_ADIM_MIN/MAX basligindaki not).
        self.silah_adim_sayisi = self._pot_adim_sayisi(self.pwm_ust_sinir)
        self._silah_adim_yayinla()

    def _pot_adim_sayisi(self, pwm):
        aralik = float(PWM_UST - PWM_ALT)
        oran = 0.0 if aralik <= 0 else (float(pwm) - PWM_ALT) / aralik
        oran = min(max(oran, 0.0), 1.0)
        return int(round(SILAH_ADIM_MIN + oran * (SILAH_ADIM_MAX - SILAH_ADIM_MIN)))

    def _silah_adim_yayinla(self):
        try:
            self.silah_adim_pub.publish(Int32(data=int(self.silah_adim_sayisi)))
        except Exception:
            pass

    def _pwm_olcekle(self, oran):
        # telemetri_sistemi.py::_pwm_olcekle ile BİREBİR aynı formül.
        if abs(oran) < 1e-6:
            return 0.0
        isaret = 1.0 if oran > 0 else -1.0
        buyukluk = min(abs(oran), 1.0)
        return isaret * (PWM_ALT + buyukluk * (self.pwm_ust_sinir - PWM_ALT))

    def _palet_yayinla(self):
        msg = Float32MultiArray()
        # İşaret kuralı arayüzdekiyle AYNI (Arduino tarafı böyle bekliyor).
        msg.data = [-float(self.anlik_sol_pwm), -float(self.anlik_sag_pwm)]
        try:
            self.palet_pub.publish(msg)
        except Exception:
            pass

    def durum_yayinla(self, veri: dict):
        """Arayüzün göstergeleri (bağlantı, buton olayları, pot, batarya)
        için - gecikmeye duyarsız, JSON olarak tek topic'te."""
        try:
            self.durum_pub.publish(String(data=json.dumps(veri)))
        except Exception:
            pass

    def tick(self):
        """Periyodik sürüş yayını + ölü adam (watchdog) kontrolü."""
        # BAĞLANTI DURUMUNU PERİYODİK TEKRARLA (2026-09-06, canlı yakalandı):
        # bağlantı olayı SADECE değişim anında yayınlanırsa ve arayüz o
        # sırada henüz hazır değilse (node arayüzden ÖNCE başlamış olabilir),
        # arayüz surus_kaynagi'nı "JOYSTICK" yapmaz ve KENDİSİ de
        # /palet_hizlari yayınlamaya devam eder -> İKİ YAYINCI çakışır
        # (ölçümde 100ms hedef yerine 60ms aralık = iki kaynak görüldü).
        # Durumu ~2sn'de bir tekrarlamak, arayüz ne zaman başlarsa başlasın
        # doğru kaynağı öğrenmesini garanti eder.
        self._durum_sayac = getattr(self, '_durum_sayac', 0) + 1
        if self._durum_sayac % 20 == 0 and getattr(self, 'panel_bagli', False):
            self.durum_yayinla({'tip': 'baglanti', 'bagli': True})
        # SILAH ADIM SAYISINI DA PERIYODIK TEKRARLA: turret_node bu node'dan
        # SONRA baslamis olabilir; pot oynatilmadikca yeni bir yayin
        # olmayacagi icin varsayilan degerde kalirdi.
        if self._durum_sayac % 20 == 0:
            self._silah_adim_yayinla()

        # ACİL STOP AKTİFKEN MANUEL'İ SÜREKLİ TEKRARLA: arayüz donmuş ya da
        # kapanmışsa bile son sözü acil stop söylesin. Arayüz sağlıklıysa
        # zaten bir tur içinde kendi durumunu MANUEL yapıyor ve çakışma
        # bitiyor (bkz. _manuele_cek). Mod kontrolünden ÖNCE, çünkü
        # aşağıdaki erken dönüş OTONOM'da bu satıra hiç gelinmemesine
        # sebep olurdu - yani tam da gerektiği durumda çalışmazdı.
        if self.acil_stop_aktif:
            try:
                self.mod_pub.publish(String(data='MANUEL'))
            except Exception:
                pass
            # SİLAH: sıfır taret komutu + ateş kapalı, HER TUR (100ms).
            # Arayüzdeki ekran butonları da /turret_manuel_cmd ve
            # /silah_manuel yayınlayabiliyor; son sözü acil stop söylesin.
            self.turret_geldi(0.0, 0.0)
            self._ates_yayinla()
            # /arac_komut'u ~2 sn'de bir TEKRARLA: araç tarafındaki
            # turret_node (veya arduino_motor_kontrol) acil stop SIRASINDA
            # yeniden başlarsa kilidi kaçırmasın. Alıcılar kenar-korumalı
            # olduğu için tekrar zararsız (log spam'i yok).
            if self._durum_sayac % 20 == 0:
                try:
                    self.komut_pub.publish(String(data='EMERGENCY_STOP_CMD'))
                except Exception:
                    pass

        # SİLAH ATEŞ HEARTBEAT'İ (bkz. ates_degisti). Mod kontrolünden
        # ÖNCE, çünkü ateş kararı araç tarafına ait (turret_node.py OTONOM
        # modda kendi kilit mantığını uygular) - panel ayrı process'e
        # taşınmadan önceki main.py davranışıyla BİREBİR aynı: anahtar
        # basılıyken koşulsuz tekrar yayınla.
        if self.ates_manuel_aktif:
            self._ates_yayinla()

        if self.arac_modu != "MANUEL":
            return
        if (self.anlik_sol_pwm != 0.0 or self.anlik_sag_pwm != 0.0):
            if (time.time() - self.son_komut_zamani) > GUVENLIK_ZAMAN_ASIMI_S:
                self.anlik_sol_pwm = 0.0
                self.anlik_sag_pwm = 0.0
                self.get_logger().warn('GÜVENLİK: joystick sinyali kesildi, paletler frenlendi')
        self._palet_yayinla()


KILIT_DOSYASI = "/tmp/tufan_kontrol_paneli_node.lock"


def _tekil_calisma_kilidi():
    """AYNI ANDA SADECE BİR node çalışmasını garanti eder.

    (2026-09-06, canlı yakalandı) Arayüz çöküp süpervizör tarafından
    yeniden başlatıldığında, önceki main.py'nin başlattığı node ORPHAN
    olarak yaşamaya devam ediyor; yeni main.py de yenisini başlatınca
    sistemde 2-3 node birikiyordu. Hepsi AYNI seri porttan okumaya
    çalıştığı için veri bozuluyor (logda aynı ACİL STOP olayının saniyede
    bir tekrarlanması bunun belirtisiydi) ve sistem yavaşlayıp arayüzü
    donduruyordu. Bu kilit, ikinci bir instance'ın sessizce çıkmasını
    sağlar - dosya kilidi process ölünce çekirdek tarafından OTOMATİK
    bırakılır, yani orphan bir kilit kalması imkânsızdır.
    """
    f = open(KILIT_DOSYASI, "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("kontrol_paneli_node: zaten çalışan bir örnek var - çıkılıyor.",
              file=sys.stderr)
        return None
    f.write(str(os.getpid()))
    f.flush()
    return f  # referans tutulmalı, kapanırsa kilit düşer


def main():
    kilit = _tekil_calisma_kilidi()
    if kilit is None:
        return
    rclpy.init()
    node = KontrolPaneliNode()

    # GUI'siz Qt olay döngüsü - QThread sinyalleri için gerekli, pencere
    # açmaz / X11'e dokunmaz.
    app = QCoreApplication(sys.argv)

    panel = KontrolPaneliThread()
    panel.surus_sinyali.connect(node.surus_geldi)
    panel.turret_sinyali.connect(node.turret_geldi)
    panel.sag_toggle_sinyali.connect(node.ates_degisti)
    panel.sol_toggle_sinyali.connect(node.acil_stop_degisti)
    panel.pot_sinyali.connect(node.pot_geldi)
    panel.baglanti_sinyali.connect(node.baglanti_degisti)
    # Sol buton (D9): ESKIDEN manuel/otonom gecisiydi, 2026-09-06'dan
    # itibaren FREN ac/kapa (bkz. node.fren_toggle).
    panel.sol_buton_sinyali.connect(node.fren_toggle)
    panel.sag_buton_sinyali.connect(
        lambda: node.durum_yayinla({'tip': 'sag_buton'}))
    panel.guvenlik_sinyali.connect(
        lambda m: node.durum_yayinla({'tip': 'log', 'mesaj': str(m)}))
    panel.yer_batarya_sinyali.connect(
        lambda m: node.durum_yayinla({'tip': 'yer_batarya', 'metin': str(m)}))
    panel.pot_sinyali.connect(
        lambda v: node.durum_yayinla({'tip': 'pot', 'pwm': int(v)}))
    # PANEL TESTİ (2026-09-06): ham veri akışı + arayüzden gelen komutlar
    # (merkezi sıfırla / ayarları uygula) için node'a panel referansı verilir.
    panel.ham_veri_sinyali.connect(node.ham_veri_geldi)
    node.panel = panel
    panel.start()

    # Sürüş yayını + watchdog
    surus_timer = QTimer()
    surus_timer.timeout.connect(node.tick)
    surus_timer.start(int(YAYIN_ARALIGI_S * 1000))

    # rclpy'yi Qt döngüsü içinde döndür (ayrı thread'e gerek yok - bu
    # process'te GUI yükü olmadığı için tek döngü fazlasıyla yeterli).
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    def kapat(*_):
        panel.durdur()
        app.quit()

    signal.signal(signal.SIGINT, kapat)
    signal.signal(signal.SIGTERM, kapat)

    try:
        app.exec_()
    finally:
        try:
            panel.durdur()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
