"""NTRIP istemcisi -> mavros RTK koprusu.

TUSAGA-Aktif (Turkiye resmi CORS agi) NTRIP caster'indan RTCM3 duzeltme
baytlarini indirip, arac Jetson'daki mavros'un zaten dinledigi
/mavros/gps_rtk/send_rtcm (mavros_msgs/RTCM) topic'ine yayinlar. mavros
bu veriyi otomatik olarak MAVLink GPS_RTCM_DATA ile Cube'a, oradan da
DroneCAN uzerinden Here4'e iletir - bu dosyanin Cube/MAVLink ile HICBIR
dogrudan iliskisi yok, sadece NTRIP'ten okuyup ROS2'ye yayinliyor.

Iki Jetson ayni ROS2 agindaki cross-machine discovery sayesinde
birbirini goruyor: bu node arac Jetson'un yayinladigi /fix (GERCEK GPS
konumu, GGA uretmek icin) topic'ine abone olur ve /mavros/gps_rtk/send_rtcm
topic'ine yayinlar - arac Jetson'daki mavros buna otomatik baglanir, ek
network kurulumu gerekmez.

Mimari notu: bu, kendi KUCUK ve IZOLE ROS2 node'unu kullanir (ayni
telemetri_sistemi.py'deki surus_node izolasyonu gibi) - baska hicbir
ROS2 abonelik/yayinla ayni node'u paylasmiyor, DDS kesif tikanmasi riski
olmasin diye.

Kullanim:
    thread = NtripRtkThread()
    thread.baglanti_sinyali.connect(...)
    thread.log_sinyali.connect(...)
    thread.start()
"""
import base64
import socket
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from mavros_msgs.msg import RTCM
    from std_msgs.msg import Header
    from sensor_msgs.msg import NavSatFix
except Exception:
    rclpy = None
    Node = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    QoSHistoryPolicy = None
    RTCM = None
    Header = None
    NavSatFix = None

from PyQt5.QtCore import QThread, pyqtSignal

# --- NTRIP caster bilgileri: TUSAGA-Aktif (Turkiye resmi CORS agi) ---
# VRSRTCM34 = "Tum Turkiye" VRS, RTCM 3.4, GPS+GLONASS+Galileo+BeiDou+QZS.
# Alternatif mountpoint'ler (sourcetable'da gorulebilir): VRSRTCM31 (RTCM 3.1,
# sadece GPS+GLONASS), FKP_RTCM31, bolgesel TG20-*-BRDCST-RTCM yayinlari.
NTRIP_HOST = "www.tusaga-aktif.gov.tr"
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "VRSRTCM34"
NTRIP_KULLANICI = "K073834001"
NTRIP_SIFRE = "SfhhZw"

MAVROS_RTCM_TOPIC = "/mavros/gps_rtk/send_rtcm"
# mavros_msgs/RTCM.data, gps_rtk eklentisi tarafindan EN FAZLA 720 bayt
# (4*180, MAVLink GPS_RTCM_DATA parcalama siniri) kabul ediyor, uzeri
# SESSIZCE atiliyor (hata yok, sadece kayboluyor) - guvenli pay icin 700.
MAKS_PARCA_BOYUTU = 700

# Arac Jetson'daki mavros'un yayinladigi GERCEK GPS konumu - bu Jetson'da
# GPS olmadigindan, VRSRTCM34 (konum tabanli VRS duzeltmesi) icin GGA/RMC
# uretmek adina buradan lat/lon/alt okunur.
ARAC_GPS_TOPIC = "/fix"

# KRITIK: VRSRTCM34 1Hz GGA+RMC bekliyor - 10sn'de bir gonderim akisi
# zayiflatiyor/kesiyor (saatlerce test edilerek dogrulandi). SADECE GGA
# yeterli olmuyor gibi gorunuyordu - RMC ile birlikte gonderilince akis
# surekli hale geldi. GERCEK konum gelene kadar (ilk birkac saniye)
# asagidaki yer tutucu (Ankara civari) kullanilir.
GGA_GONDERIM_ARALIGI_SN = 1.0
VARSAYILAN_ENLEM = 39.9334
VARSAYILAN_BOYLAM = 32.8597

# Cok agresif "stale" tespiti alicinin RTK Fixed kilitlenmesini surekli
# sifirliyor (test edildi) - sadece gercekten uzun sure veri gelmezse
# yeniden baglan.
STALE_ESIK_SN = 75.0
YENIDEN_BAGLANMA_BEKLEME_SN = 5.0


def _gga_uret(enlem, boylam, yukseklik_m=850.0):
    """Standart NMEA GGA cumlesi (checksum dahil) uretir."""
    def derece_to_nmea(deger, pozitif_harf, negatif_harf):
        yon = pozitif_harf if deger >= 0 else negatif_harf
        deger = abs(deger)
        derece = int(deger)
        dakika = (deger - derece) * 60.0
        return derece, dakika, yon

    su_an = time.gmtime()
    zaman_str = time.strftime("%H%M%S", su_an) + ".00"

    lat_d, lat_m, lat_yon = derece_to_nmea(enlem, "N", "S")
    lon_d, lon_m, lon_yon = derece_to_nmea(boylam, "E", "W")

    govde = (
        f"GPGGA,{zaman_str},{lat_d:02d}{lat_m:07.4f},{lat_yon},"
        f"{lon_d:03d}{lon_m:07.4f},{lon_yon},1,08,0.9,{yukseklik_m:.1f},M,0.0,M,,"
    )
    checksum = 0
    for karakter in govde:
        checksum ^= ord(karakter)
    return f"${govde}*{checksum:02X}\r\n"


def _rmc_uret(enlem, boylam):
    """Standart NMEA RMC cumlesi uretir (checksum dahil).

    TUSAGA-Aktif'in VRSRTCM34 mountpoint'i icin SADECE GGA yeterli
    olmuyordu - RMC ile birlikte gonderilince akis surekli hale geldi
    (saatlerce test edilerek bulundu). Hiz/rota bilgimiz olmadigindan
    0.0 gonderiliyor - VRS konumlandirmasi icin onemli olan enlem/boylam.
    """
    def derece_to_nmea(deger, pozitif_harf, negatif_harf):
        yon = pozitif_harf if deger >= 0 else negatif_harf
        deger = abs(deger)
        derece = int(deger)
        dakika = (deger - derece) * 60.0
        return derece, dakika, yon

    su_an = time.gmtime()
    zaman_str = time.strftime("%H%M%S", su_an) + ".00"
    tarih_str = time.strftime("%d%m%y", su_an)

    lat_d, lat_m, lat_yon = derece_to_nmea(enlem, "N", "S")
    lon_d, lon_m, lon_yon = derece_to_nmea(boylam, "E", "W")

    govde = (
        f"GPRMC,{zaman_str},A,{lat_d:02d}{lat_m:07.4f},{lat_yon},"
        f"{lon_d:03d}{lon_m:07.4f},{lon_yon},0.0,0.0,{tarih_str},,,A"
    )
    checksum = 0
    for karakter in govde:
        checksum ^= ord(karakter)
    return f"${govde}*{checksum:02X}\r\n"


class _ParcaliCozucu:
    """HTTP 'Transfer-Encoding: chunked' cercevesini soket akisindan coker.

    Skylark HTTP/1.x yaniti verdiginde chunked olabilir - cozulmezse ham
    RTCM verisi hex-uzunluk satirlari ve "\\r\\n" ayraclariyla kirlenir.
    besle() soketten gelen ham baytlari alir, sadece cozulmus (gercek
    RTCM) baytlari dondurur.
    """

    def __init__(self):
        self._arabellek = b""
        self._kalan_parca_boyutu = 0
        # Bir onceki besle() cagrisinda parca verisi tam tuketildi ama
        # ardindan gelmesi gereken "\r\n" henuz arabellekte yoktu - bu
        # bayrak olmadan bir sonraki cagrida o "\r\n" yanlislikla uzunluk
        # satiri olarak yorumlanip cozucu kilitlenebiliyordu (bulundu).
        self._kuyrukta_crlf_bekleniyor = False

    def besle(self, veri: bytes) -> bytes:
        self._arabellek += veri
        cikti = b""
        while True:
            if self._kuyrukta_crlf_bekleniyor:
                if len(self._arabellek) < 2:
                    break  # parca-sonu "\r\n" henuz tam gelmedi
                self._arabellek = self._arabellek[2:]
                self._kuyrukta_crlf_bekleniyor = False
                continue
            if self._kalan_parca_boyutu > 0:
                alinacak = min(self._kalan_parca_boyutu, len(self._arabellek))
                cikti += self._arabellek[:alinacak]
                self._arabellek = self._arabellek[alinacak:]
                self._kalan_parca_boyutu -= alinacak
                if self._kalan_parca_boyutu > 0:
                    break  # bu parcanin geri kalani henuz gelmedi
                self._kuyrukta_crlf_bekleniyor = True
                continue
            satir_sonu = self._arabellek.find(b"\r\n")
            if satir_sonu == -1:
                break  # uzunluk satiri henuz tam gelmedi, daha fazla bekle
            uzunluk_satiri = self._arabellek[:satir_sonu]
            self._arabellek = self._arabellek[satir_sonu + 2:]
            try:
                boyut = int(uzunluk_satiri.split(b";")[0].strip(), 16)
            except ValueError:
                continue  # bicimsiz satiri at, devam et
            if boyut == 0:
                break  # bitis parcasi ("0\r\n\r\n") - akis kapanacak
            self._kalan_parca_boyutu = boyut
        return cikti


class NtripRtkThread(QThread):
    baglanti_sinyali = pyqtSignal(bool)
    log_sinyali = pyqtSignal(str)

    def __init__(self, host=None, port=None, mountpoint=None, kullanici=None, sifre=None):
        super().__init__()
        self.host = host or NTRIP_HOST
        self.port = port or NTRIP_PORT
        self.mountpoint = mountpoint or NTRIP_MOUNTPOINT
        self.kullanici = kullanici or NTRIP_KULLANICI
        self.sifre = sifre or NTRIP_SIFRE
        self._calisiyor = True
        self._node = None
        self._pub = None
        self._enlem = VARSAYILAN_ENLEM
        self._boylam = VARSAYILAN_BOYLAM
        self._yukseklik = 850.0

    def yapilandirilmis_mi(self):
        return "DOLDURULMADI" not in (self.host, self.mountpoint, self.kullanici, self.sifre)

    def guncel_konum_ayarla(self, enlem, boylam):
        # main.py'deki harita_sistemi Qt sinyaliyle de cagirilabilir (yedek
        # yol) - asil kaynak asagidaki _gps_callback (dogrudan ROS2
        # aboneligi, arac Jetson'dan cross-machine).
        self._enlem = enlem
        self._boylam = boylam

    def _gps_callback(self, msg):
        # Arac Jetson'daki mavros'un yayinladigi GERCEK GPS konumu
        # (cross-machine ROS2 discovery ile dogrudan bu node'a geliyor).
        self._enlem = msg.latitude
        self._boylam = msg.longitude
        if msg.altitude:
            self._yukseklik = msg.altitude

    def durdur(self):
        self._calisiyor = False
        self.quit()
        self.wait()

    def _ros_baglanti_kur(self):
        if rclpy is None or RTCM is None:
            self.log_sinyali.emit("⚠️ mavros_msgs bulunamadı, RTK yayıncısı devre dışı.")
            return False
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('tufan_ntrip_rtk_koprusu')
        rtcm_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self._node.create_publisher(RTCM, MAVROS_RTCM_TOPIC, rtcm_qos)
        if NavSatFix is not None:
            self._node.create_subscription(NavSatFix, ARAC_GPS_TOPIC, self._gps_callback, 10)
        return True

    def _rtcm_yayinla(self, veri: bytes):
        if self._pub is None or not veri:
            return
        for i in range(0, len(veri), MAKS_PARCA_BOYUTU):
            parca = veri[i:i + MAKS_PARCA_BOYUTU]
            msg = RTCM()
            msg.header = Header()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = "ntrip"
            msg.data = list(parca)
            self._pub.publish(msg)

    def _ntrip_baglan(self):
        # NOT: basit NTRIP v1 istek formati - HTTP/1.0, Host/Ntrip-Version/
        # Connection basliklari YOK, birebir bu sirayla. Hem Skylark'ta hem
        # TUSAGA-Aktif'te calistigi canli test edilerek dogrulandi.
        soket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        soket.settimeout(10.0)
        soket.connect((self.host, self.port))

        kimlik = base64.b64encode(f"{self.kullanici}:{self.sifre}".encode()).decode()
        istek = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP SimpleClient/1.0\r\n"
            f"Accept: */*\r\n"
            f"Authorization: Basic {kimlik}\r\n"
            f"\r\n"
        )
        soket.sendall(istek.encode())

        # Yanit iki farkli bicimde gelebilir:
        #  - NTRIP v1/ICY: "ICY 200 OK\r\n" TEK basina, hemen ardindan
        #    binary RTCM baslar (ikinci bir "\r\n" beklemek sonsuza kadar
        #    takilmaya sebep olur, cunku o ikinci CRLF hic gelmez).
        #  - HTTP/1.x: standart baslik blogu, "\r\n\r\n" ile biter.
        baslik_verisi = b""
        while True:
            parca = soket.recv(1)
            if not parca:
                raise ConnectionError("NTRIP sunucusu baglantiyi erken kapatti")
            baslik_verisi += parca
            if baslik_verisi == b"ICY 200 OK\r\n":
                break
            if b"\r\n\r\n" in baslik_verisi:
                break
            if len(baslik_verisi) > 4096:
                raise ConnectionError("NTRIP basligi cok uzun/bicimsiz")

        baslik_metni = baslik_verisi.decode(errors="ignore")
        ilk_satir = baslik_metni.splitlines()[0] if baslik_metni else ""
        if "200" not in ilk_satir:
            raise ConnectionError(f"NTRIP sunucusu reddetti: {ilk_satir.strip()}")

        parcali_mi = "transfer-encoding: chunked" in baslik_metni.lower()

        soket.settimeout(2.0)
        return soket, parcali_mi

    def run(self):
        if not self.yapilandirilmis_mi():
            self.log_sinyali.emit(
                "ℹ️ NTRIP/RTK bilgileri henüz girilmedi (ntrip_rtk_sistemi.py "
                "üstündeki NTRIP_HOST/MOUNTPOINT/KULLANICI/SIFRE) - RTK "
                "düzeltmesi gönderilmeyecek."
            )
            return

        if not self._ros_baglanti_kur():
            return

        while self._calisiyor:
            soket = None
            try:
                soket, parcali_mi = self._ntrip_baglan()
                cozucu = _ParcaliCozucu() if parcali_mi else None
                if parcali_mi:
                    self.log_sinyali.emit("ℹ️ Sunucu 'chunked' aktarım kullanıyor, RTCM baytları çözülerek yayınlanacak.")

                # Ilk GGA+RMC'yi hemen gonder, sonra 1Hz'de tekrarla (KRITIK -
                # 10sn'de bir yeterli degil, akisi zayiflatiyor/kesiyor;
                # SADECE GGA da yeterli olmuyordu, RMC ile birlikte gonderilince
                # akis surekli hale geldi).
                soket.sendall(_gga_uret(self._enlem, self._boylam, self._yukseklik).encode())
                soket.sendall(_rmc_uret(self._enlem, self._boylam).encode())
                son_gga_zamani = time.time()
                son_veri_zamani = time.time()

                self.baglanti_sinyali.emit(True)
                self.log_sinyali.emit(f"🛰️ NTRIP bağlandı: {self.host}:{self.port}/{self.mountpoint}")

                while self._calisiyor:
                    rclpy.spin_once(self._node, timeout_sec=0.0)  # yeni GPS okumalarini isle

                    if time.time() - son_gga_zamani >= GGA_GONDERIM_ARALIGI_SN:
                        soket.sendall(_gga_uret(self._enlem, self._boylam, self._yukseklik).encode())
                        soket.sendall(_rmc_uret(self._enlem, self._boylam).encode())
                        son_gga_zamani = time.time()

                    try:
                        veri = soket.recv(4096)
                    except socket.timeout:
                        # Kisa sureli veri gelmemesi normal - sadece
                        # GERCEKTEN uzun sure (STALE_ESIK_SN) veri yoksa
                        # yeniden baglan (agresif reconnect RTK Fixed
                        # kilitlenmesini sifirliyor, test edildi).
                        if time.time() - son_veri_zamani > STALE_ESIK_SN:
                            raise ConnectionError(
                                f"{STALE_ESIK_SN:.0f}sn'dir veri gelmedi (stale)"
                            )
                        continue
                    if not veri:
                        raise ConnectionError("NTRIP bağlantısı sunucu tarafından kapatıldı")

                    son_veri_zamani = time.time()
                    self._rtcm_yayinla(cozucu.besle(veri) if cozucu else veri)
            except Exception as e:
                self.baglanti_sinyali.emit(False)
                self.log_sinyali.emit(
                    f"⚠️ NTRIP bağlantı hatası: {e} - "
                    f"{YENIDEN_BAGLANMA_BEKLEME_SN:.0f}sn sonra tekrar denenecek"
                )
                time.sleep(YENIDEN_BAGLANMA_BEKLEME_SN)
            finally:
                if soket is not None:
                    try:
                        soket.close()
                    except Exception:
                        pass
