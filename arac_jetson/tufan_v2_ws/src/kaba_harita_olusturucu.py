#!/usr/bin/env python3
"""Kaba/kalici UZUN-ENGEL haritasi (2026-09-02, kullanici istegi).

NEDEN: rampa/dik engel gecisinde LIDAR duzlemi govdeyle birlikte egilince
gercek bir bariyeri ANLIK olarak kacirabiliyor (2D LIDAR'in fiziksel
siniri - bkz. egim_costmap_ayarlayici.py'nin ayni konudaki notu).
Kullanicinin onerisi: parkuru once MANUEL surup gercek/uzun engelleri
(bariyer, duvar) bir kere DUZGUN goruculu HAFIZAYA alalim; otonom turda bu
hafiza, anlik LIDAR'in gecici kor noktalarindan ETKILENMEYEN IKINCI bir
kaynak olarak Nav2'nin costmap'ine (static_layer) beslenir. Mevcut reaktif
obstacle_layer HICBIR SEKILDE degismiyor/degistirilmiyor - dubalar/koniler
HALA sadece o katmanda, gercek zamanli olarak gorunur/temizlenir.

PITCH ISTISNASI (2026-09-02, IKINCI canli testte bulundu): bu node ilk
surumde pitch'i HIC kontrol etmiyordu - manuel turda rampa gecilince
zemin GENIS bir acisal yelpazede SUREKLI/yumusak degisen mesafeler
uretip UZUN_ENGEL_ESIGI_M'yi kolayca asti, harita ince duvar cizgileri
yerine DEV, kalici bir hayalet "yumru" oldu (bkz. pitch_esigi_derece
parametresinin ustundeki not) - normal costmap'teki GECICI pitch
hatasindan cok daha kotu bir sonuc, cunku bu haritada SONSUZA KADAR
kalirdi. Artik egim_costmap_ayarlayici.py/serbest_yon_takipcisi.py ile
AYNI pitch esigini kullanip rampa gecisinde tick'i TAMAMEN atlar.

DUBALAR/KONILER NEDEN HARIC TUTULUYOR: yerleri degisebilir - kalici
hafizaya girerlerse eski konumda HAYALET ENGEL olarak sikismaya yol
acarlar (tam da onlemeye calistigimiz sorun). Ayrim FIZIKSEL UZUNLUK ile
yapilir: bir bariyer/duvar taramada UZUN, surekli bir nokta dizisi olarak
gorunur; bir duba/koni KISA, nokta-benzeri bir kume olarak gorunur (bkz.
UZUN_ENGEL_ESIGI_M - CANLI TESTTE ayarlanmasi gereken bir TAHMIN).

TASARIM FELSEFESI (bilincli tercih): slam_toolbox/nav2_map_server/nav2_amcl
bu sistemde KURULU ama KULLANILMADI - hepsi TUM sahneyi (dubalar dahil)
haritalar, dubalari BITMIS bir occupancy grid'den geriye donuk filtrelemek
zor/guvenilmezken, TARAMA ANINDA (nokta bulutu hala ayrikken) filtrelemek
COK daha kolay. Ayni sebeple tam bir SLAM/AMCL lokalizasyon yigini da
EKLENMEDI - tufan_mppi.launch.py'nin "haritasiz calisiyoruz" felsefesi
KORUNUYOR: konum_birlestirici.py'nin fused /odom cercevesi MANUEL ve
OTONOM turlar arasinda ASLA resetlenmiyor (mod degisimi sadece bir GCS
topic mesaji, ROS graph'i surekli calisir durumda kaliyor), bu yuzden
harita da AYNI /odom cercevesinde tutarli kalir - ayri bir lokalizasyon
adimina GEREK YOK.

MOD KONTROLU: /surus_modu (String, "MANUEL"/"OTONOM" - surus_koprusu.py
ile AYNI sozlesme, harici GCS uygulamasindan gelir) "MANUEL" iken AKTIF
kumeleme/biriktirme yapar. "OTONOM"a gecince biriktirme DURUR (harita
"donar") - otonom turdaki dubalarin/gecici degisikliklerin haritaya
KARISMAMASI icin ayrica bir mekanizma GEREKMEDI, mod kontrolu zaten
yeterli. Son biriktirilen harita, YAYIN_PERIYODU_S periyoduyla surekli
yayinlanmaya devam eder (TRANSIENT_LOCAL QoS - gec baslayan static_layer
bile son haritayi alir).

KUMELEME: serbest_yon_takipcisi.py'deki _en_genis_gap() ile AYNI ardisik-
gruplama desenini kullanir (bu projede paylasilan modul yok, dosya ici
kopyalanir - bkz. o dosyanin ayni konudaki notlari) FAKAT ACIK degil DOLU
noktalari gruplar, VE ayrica BUYUK MESAFE SICRAMALARINDA da grubu boler
(iki farkli fiziksel nesne acisal olarak bitisik ama farkli derinlikte
olabilir - _en_genis_gap bunu yapmiyordu cunku onun amaci farkliydi,
sadece "acik mi degil mi" onemliydi, burada GERCEK NESNE SINIRLARI
onemli).

COKLU-TIK DOGRULAMA: bir hucre sadece HIT_ESIGI farkli tick'te "uzun
engel" olarak gorulunce KESINLESIR ve yayinlanan haritaya girer - tek bir
yanlis siniflandirmanin (orn. yan yana duran birkac duba bir an icin
"uzun" gorunmesi, ya da bir bariyere COK dar/egik acidan bakilip yanlislikla
kisa gorunmesi) kalici hayalet engel birakmasini ya da gercek bir bariyeri
kacirmasini onler - arac manuel surus sirasinda ayni nesneyi DOGAL olarak
birden fazla acidan/mesafeden gorur.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String


def _on_merkezli(ham_aci_rad):
    """Ham scan acisini (radyan, TF geregi ±180 civari = ON) 0=tam-on
    olacak sekilde kaydirir ve -pi..pi'ye normalize eder (serbest_yon_
    takipcisi.py'deki AYNI fonksiyonun kopyasi)."""
    a = ham_aci_rad - math.pi
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pitch_of(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


class KabaHaritaOlusturucu(Node):
    def __init__(self):
        super().__init__('kaba_harita_olusturucu')

        # KESIN COZUM (2026-09-02, CANLI TESTTE BULUNDU): ilk manuel tur
        # denemesinde harita NEREDEYSE BOS cikti (14 hucre, tek kucuk
        # segment) - 1.5m (gap-following'in ACIK_MESAFE_ESIGI_M'inden
        # KOPYALANMIS, YANLIS amac icin) cok kisaydi: normal surus
        # sirasinda arac bariyerlere genelde bundan daha UZAK kaliyor
        # (duvara yapismadan surmek normal), bu yuzden coğu gerçek bariyer
        # DOLU adayi bile OLAMADI. local_costmap'in gercek obstacle_layer'i
        # (nav2_params.yaml) 6.0m'ye kadar gercek engel sayiyor - AYNI
        # sinir burada da kullanildi (LIDAR'in "guvenilir engel" menzili
        # icin zaten kanitlanmis bir deger).
        self.declare_parameter('dolu_mesafe_esigi_m', 6.0)
        self.declare_parameter('mesafe_sicrama_esigi_m', 0.4)
        self.declare_parameter('uzun_engel_esigi_m', 1.2)
        self.declare_parameter('grid_cozunurluk_m', 0.1)
        self.declare_parameter('hit_esigi', 3)
        self.declare_parameter('yayin_periyodu_s', 1.0)
        # KESIN COZUM (2026-09-02, CANLI TESTTE BULUNDU - IKINCI manuel tur
        # denemesi, 6.0m esigiyle): harita ince duvar/bariyer CIZGILERI
        # yerine DEV, dolgun tek bir "yumru" olarak cikti (~%40'i dolu
        # isaretlenmisti). KOK NEDEN: egim_costmap_ayarlayici.py VE
        # serbest_yon_takipcisi.py'nin AYNI fiziksel nedenle korundugu
        # sorun - 2D LIDAR duzlemi govdeyle birlikte egilince (rampa/
        # egim gecisi) ZEMINI engel sanar. O iki dosya BUNU zaten
        # ele aliyordu ama bu YENI node (kaba_harita_olusturucu) pitch'i
        # HIC kontrol etmiyordu - rampa gecisinde zemin, GENIS bir acisal
        # yelpazede SUREKLI/yumusak degisen mesafeler urettigi icin
        # UZUN_ENGEL_ESIGI_M'yi kolayca asip KALICI bir hayalet "duvar"
        # olarak haritaya YANLISLIKLA yakiliyordu (rampa gecisi ANLIK bir
        # LIDAR hatasiyken, bu haritada SONSUZA KADAR kaliyordu - normal
        # costmap'teki gecici hatadan cok daha kotu bir sonuc). Cozum:
        # AYNI esik (PITCH_ESIGI_DERECE, diger iki dosyayla TUTARLI) - egim
        # asilinca bu tick TAMAMEN atlanir, hicbir sekilde biriktirme
        # yapilmaz.
        self.declare_parameter('pitch_esigi_derece', 3.0)
        self._pitch_esigi = math.radians(self.get_parameter('pitch_esigi_derece').value)

        self._dolu_esik = self.get_parameter('dolu_mesafe_esigi_m').value
        self._sicrama_esik = self.get_parameter('mesafe_sicrama_esigi_m').value
        self._uzun_esik = self.get_parameter('uzun_engel_esigi_m').value
        self._cozunurluk = self.get_parameter('grid_cozunurluk_m').value
        self._hit_esigi = self.get_parameter('hit_esigi').value

        # Guvenli varsayilan: mod bilgisi gelene kadar MANUEL kabul ETME -
        # surus_koprusu.py'nin TERSI mantik gerekiyor burada (o "bilinmeyen
        # durumda otonom SURUS varsayma" der - guvenlik icin). Burada ise
        # "bilinmeyen durumda YANLISLIKLA biriktirmeye baslama" istiyoruz,
        # cunku /surus_modu ilk mesaji gelene kadar OTONOM SURUS SIRASINDA
        # da biriktirmeye baslarsa dubalar kalici haritaya karisabilir.
        self._surus_modu = 'BILINMIYOR'
        self._son_odom = None

        self._hit_sayaclari = {}  # (gx, gy) -> int, kac FARKLI tick'te dolu gorulduğü
        self._kesinlesmis = set()  # (gx, gy), HIT_ESIGI'ye ulasmis (kesin) hucreler

        self.create_subscription(String, '/surus_modu', self._mod_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        harita_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._harita_pub = self.create_publisher(OccupancyGrid, '/kaba_harita', harita_qos)
        self.create_timer(self.get_parameter('yayin_periyodu_s').value, self._harita_yayinla)

        self.get_logger().info(
            'kaba_harita_olusturucu aktif: SADECE /surus_modu=MANUEL iken '
            f'uzun engelleri (>{self._uzun_esik}m, {self._hit_esigi} tik dogrulamali) '
            'biriktirir, OTONOM moda gecince donar. /kaba_harita olarak yayinlanir.')

    def _mod_cb(self, msg: String):
        yeni = msg.data.strip().upper()
        if yeni != self._surus_modu and yeni in ('MANUEL', 'OTONOM'):
            self.get_logger().info(
                f'kaba_harita_olusturucu modu -> {yeni} '
                f'({"biriktirme AKTIF" if yeni == "MANUEL" else "DONDU (biriktirme durdu)"})')
        self._surus_modu = yeni

    def _odom_cb(self, msg: Odometry):
        self._son_odom = msg

    def _scan_cb(self, msg: LaserScan):
        if self._surus_modu != 'MANUEL' or self._son_odom is None:
            return

        # PITCH ISTISNASI (bkz. dosya basi + pitch_esigi_derece notu) -
        # egim/rampa gecisinde LIDAR zemini engel sanabilir, bu tick
        # TAMAMEN atlanir (biriktirme YOK).
        pitch = _pitch_of(self._son_odom.pose.pose.orientation)
        if abs(pitch) > self._pitch_esigi:
            return

        dolu = []  # (on_merkezli_aci, mesafe) - sadece DOLU_MESAFE_ESIGI_M altindakiler
        n = len(msg.ranges)
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r >= self._dolu_esik:
                continue
            ham_aci = msg.angle_min + i * msg.angle_increment
            dolu.append((_on_merkezli(ham_aci), r))

        if not dolu:
            return
        dolu.sort(key=lambda t: t[0])

        # Ardisik dolu noktalari gruplara ayir: aci-sureklilik KOPARSA (normal
        # ornekleme araligindan buyukse) VEYA mesafe ANIDEN SICRARSA (farkli
        # bir fiziksel nesneye gecildiyse) grup boler.
        aci_artis = abs(msg.angle_increment)
        bosluk_esigi = max(aci_artis * 3.0, math.radians(2.0))
        gruplar = []
        mevcut = [dolu[0]]
        for k in range(1, len(dolu)):
            onceki_aci, onceki_r = mevcut[-1]
            aci, r = dolu[k]
            if (aci - onceki_aci) <= bosluk_esigi and abs(r - onceki_r) <= self._sicrama_esik:
                mevcut.append((aci, r))
            else:
                gruplar.append(mevcut)
                mevcut = [(aci, r)]
        gruplar.append(mevcut)

        p = self._son_odom.pose.pose.position
        yaw = _yaw_of(self._son_odom.pose.pose.orientation)

        bu_tick_dokunulan = set()
        for g in gruplar:
            if len(g) < 2:
                continue  # tek nokta - uzunluk hesaplanamaz, kesinlikle duba/koni adayi
            a0, r0 = g[0]
            a1, r1 = g[-1]
            x0, y0 = r0 * math.cos(a0), r0 * math.sin(a0)
            x1, y1 = r1 * math.cos(a1), r1 * math.sin(a1)
            uzunluk = math.hypot(x1 - x0, y1 - y0)
            if uzunluk < self._uzun_esik:
                continue  # duba/koni adayi - ATLA, HICBIR SEKILDE biriktirme

            for aci, r in g:
                aci_odom = yaw + aci
                gx = int(round((p.x + r * math.cos(aci_odom)) / self._cozunurluk))
                gy = int(round((p.y + r * math.sin(aci_odom)) / self._cozunurluk))
                hucre = (gx, gy)
                if hucre in bu_tick_dokunulan:
                    continue  # ayni tick icinde iki kez sayilmasin
                bu_tick_dokunulan.add(hucre)
                yeni_sayac = self._hit_sayaclari.get(hucre, 0) + 1
                self._hit_sayaclari[hucre] = yeni_sayac
                if yeni_sayac >= self._hit_esigi:
                    self._kesinlesmis.add(hucre)

    def _harita_yayinla(self):
        if not self._kesinlesmis:
            return

        xs = [c[0] for c in self._kesinlesmis]
        ys = [c[1] for c in self._kesinlesmis]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        genislik = max_x - min_x + 1
        yukseklik = max_y - min_y + 1

        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self._cozunurluk)
        msg.info.width = genislik
        msg.info.height = yukseklik
        msg.info.origin.position.x = min_x * self._cozunurluk
        msg.info.origin.position.y = min_y * self._cozunurluk
        msg.info.origin.orientation.w = 1.0

        # -1 = BILINMIYOR (varsayilan doldurma) - 0 (bos/kesin temiz) DEGIL,
        # cunku bu kaba haritanin KESINLESMEMIS hucreleri hakkinda hicbir
        # iddiasi yok; sadece KESIN gordugu uzun-engel hucrelerini (100)
        # bildirir. Boylece static_layer, bilmedigi alanlarda obstacle_
        # layer'in gercek-zamanli kararini EZMEZ.
        data = [-1] * (genislik * yukseklik)
        for (gx, gy) in self._kesinlesmis:
            idx = (gy - min_y) * genislik + (gx - min_x)
            data[idx] = 100
        msg.data = data
        self._harita_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KabaHaritaOlusturucu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
