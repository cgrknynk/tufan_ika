#!/usr/bin/env python3
"""Aracin ONUNE, LIDAR'in GERCEKTEN BOS gordugu yere otomatik hedef atar.

2026-09-06, kullanici istegi: "otonom kodunda arac odometrisi kaydiginda
attigim noktalarda kayiyor; bunun yerine arac onune noktalar atarak
giderse konum tahmini kaymasindan dolayi otonom surus etkilenmez ve
engelleri ortalayarak gider. Onune nokta atarken lidarin on tarafini
kullansin, bos buldugu yerlere atsin - boylelikle engel ustune nokta atma
sorunu cozulur."

=======================================================================
NEDEN KAYMA SORUNUNU COZER
=======================================================================
Elle atilan (ya da bir kez atilip uzun sure yasayan) hedefler 'odom'
cercevesinde SABIT durur; odometri kayinca arac ile hedef arasindaki
GERCEK geometri bozulur - hedef fiziksel dunyada baska bir yere gider.
Bu dugum hedefi HER TICK'TE (varsayilan 4Hz) O ANKI LIDAR taramasindan
YENIDEN uretir. Hedef 'odom'a sadece gonderilirken cevrilir ve omru
saniyenin altindadir - o kisa surede birikebilecek kayma santimetre
mertebesindedir. Yani kayma DUZELTILMEZ, ONEMSIZLESTIRILIR.

=======================================================================
PARKUR OZELINDE COZULEN 2D LIDAR TUZAKLARI
=======================================================================
Kullanicinin saydigi parkur ogeleri: dubalar arasi slalom, 3 metrelik
bariyerlerden olusan koridor, donusler, YAN EGIM, DIK EGIM (rampa) ve
DIK ENGEL. 2D LIDAR tek duzlemde tarar ve o duzlem GOVDEYLE BIRLIKTE
egilir - bu, asagidaki UC ayri yanilsamayi uretir:

  (A) INISTE ZEMINI DUVAR SANMA. Burun asagi egilince tarama duzlemi
      birkac metre ilerideki DUZ ZEMINE carpar; on taraf "duvar" gibi
      gorunur.
  (B) YAN EGIMDE bir yanin zemine, diger yanin GOKYUZUNE bakmasi.
      Asagi kalan yan sahte engel, yukari kalan yan sahte BOSLUK uretir.
      Naif bir "en acik yone git" mantigi tam da bu sahte bosluga
      yonelir - sahada bariyerin USTUNE tirmanma bu sekilde olusmustu
      (bkz. serbest_yon_takipcisi.py'nin ayni tespiti).
  (C) TIRMANIRKEN her yeri bos gorme. Burun yukari egilince isinlar
      alcak engellerin USTUNDEN gecer; yanlar "bos" gorunur.

(A) ve (B) icin FIZIKSEL, kesin bir cozum var: her isinin ZEMINE hangi
mesafede carpacagi, govdenin GERCEK yonelimi (odom quaternion'u) ve
LIDAR yuksekligi bilindiginde HESAPLANABILIR. Bu mesafeye ulasan ya da
onu gecen her okuma ZEMINDIR (daha otesi fiziken gorulemez), engel
degildir - _zemin_mesafesi() bunu isin isin hesaplar ve o okumalari
atar. Hesap Euler acisi cikarmadan, dogrudan quaternion'un donme
matrisinin UCUNCU SATIRI ile yapilir; boylece roll/pitch isaret
konvansiyonu (bu projede tekrar tekrar sorun cikaran bir konu)
denklemin disinda kalir.

(C) icin YAZILIMSAL COZUM YOKTUR - sensor o bilgiyi hic toplamiyor.
Tek dogru davranis, tirmanirken YANLARA GUVENMEMEK: fan daraltilir ve
arac mevcut yonunde kisa adimlarla ilerler.

  (D) YOKUSU/DIK ENGELI DUVAR SANMA. Rampanin on yuzu duz zeminde
      dururken GERCEKTEN duvar gibi gorunur - bu bir yanilsama degil,
      dogru bir olcumdur; yanlis olan CIKARIMDIR ("cikmaz, don"). Bu
      parkurda cikmaz sokak YOK; on taraf TAMAMEN kapaliysa ve kapatan
      sey govdeye DIK, DUZ bir yuzeyse (bkz. _rampa_yuzeyi_mi) bu
      buyuk ihtimalle tirmanilacak rampadir. Bu durumda arac yuzeye DIK
      ve KISA bir hedefle yaklasir; govde egilmeye baslayinca (C)
      moduna gecer. Bu mod parametreyle kapatilabilir (rampa_modu_acik).

=======================================================================
HEDEF SECIMI (slalom + koridor ortalama + donusler TEK mekanizmada)
=======================================================================
Aci taramasi (fan) ile aday yonler denenir. Her aday yon icin:
  1) O yonde aracin GENISLIGI kadar bir koridor supurulur; koridora
     giren en yakin engel, gidilebilecek mesafeyi (d) belirler.
  2) Hedef noktasinin TUM engellere olan en kisa mesafesi = aciklik.
Puan = aciklik + katedilen mesafe - duz-gitme tercihi - histerezis.

Bu TEK puanlama uc davranisi da kendiliginden uretir:
  * KORIDOR ORTALAMA: 3m'lik bariyer koridorunda acikligi en buyuk
    nokta tam ORTADIR - ayrica "ortala" kurali yazmaya gerek yok.
  * SLALOM: bir dubanin iki yaninda da gecis varsa, aciklik daha
    genis olan taraf kazanir; histerezis terimi de aracin karar
    verdigi tarafta KALMASINI saglar (dubanin onunde saga-sola
    salinim, bu terim olmadan olusan tipik hatadir).
  * DONUSLER: duz yon kapaninca o adayin mesafesi kisalir, acik
    tarafin puani dogal olarak one gecer.

GUVENLIK: hicbir aday minimum hedef mesafesini saglamiyorsa (ve rampa
yuzeyi de degilse) HICBIR HEDEF GONDERILMEZ - arac mevcut hedefini
bitirip durur. Yanlis yone hedef atmaktansa durmak tercih edilir.

AKTIVASYON: varsayilan olarak PASIFTIR. /oto_nokta_aktif (Bool) ile
elle acilir (saha testi icin). Gorev sekansinin /otonom_surus_aktif
sinyaliyle otomatik aktiflesmesi, DOGRULANANA KADAR kapalidir -
'otonom_surus_ile_aktiflesir' parametresi True yapilarak acilir.

TESHIS: /oto_nokta_durum (String, JSON) her tick yayinlanir - secilen
mod, aci, mesafe, aciklik, elenen zemin isini sayisi. Yer istasyonundan
`ros2 topic echo /oto_nokta_durum` ile canli izlenebilir.

*** SAHADA DOGRULANMASI GEREKENLER ***
- lidar_yukseklik_m / lidar_ileri_ofset_m URDF'ten (base_footprint ->
  lidar_link = 0.575m yukseklik, 0.5m ileri) alindi; TF ile teyit et:
  `ros2 run tf2_ros tf2_echo base_footprint lidar_link`
- arac_yari_genislik_m = 0.60 (URDF govde 1.1m + tekerler ±0.575).
- Aci isareti: + = SOL kabul edildi (ROS standardi, serbest_yon_
  takipcisi.py de ayni varsayimi kullaniyor). Arac ters yone
  kaciyorsa ONCE bunu kontrol et.
"""
import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


def _on_merkezli(ham_aci_rad):
    """Ham scan acisini 0=tam-on olacak sekilde kaydirir.

    scan_front_filter.py'nin notu: base_footprint -> lidar_link net 180
    derece donuk, yani GERCEK ON ham acida ±180 civaridir. Bu fonksiyon
    serbest_yon_takipcisi.py'dekiyle BIREBIR ayni (tutarlilik)."""
    a = ham_aci_rad - math.pi
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _donme_matrisi_ucuncu_satir(q):
    """Quaternion'un donme matrisinin 3. satiri: bir GOVDE vektorunun
    DUNYA-z bilesenini verir.

    Euler acisi (roll/pitch) cikarip yeniden birlestirmek yerine bunu
    kullaniyoruz - bu projede pitch/roll ISARET konvansiyonu defalarca
    sorun cikardi (bkz. konum_birlestirici.py IMU montaj duzeltmesi,
    serbest_yon_takipcisi.py ROLL_DUZELTME_YONU notu). Bu satir
    konvansiyondan BAGIMSIZ olarak dogrudur; yaw'in z-bilesenine etkisi
    olmadigi icin yaw'in dahil olmasi da zararsizdir."""
    return (2.0 * (q.x * q.z - q.w * q.y),
            2.0 * (q.y * q.z + q.w * q.x),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y))


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class OnBoslukNoktaAtici(Node):
    def __init__(self):
        super().__init__('on_bosluk_nokta_atici')

        # --- Geometri (URDF'ten; TF ile dogrulanmali - bkz. dosya basi) ---
        self.declare_parameter('lidar_yukseklik_m', 0.575)
        self.declare_parameter('lidar_ileri_ofset_m', 0.5)
        self.declare_parameter('arac_yari_genislik_m', 0.60)
        self.declare_parameter('guvenlik_payi_m', 0.25)

        # --- Aday taramasi ---
        # KAPALI DONGU SIMULASYONUNDA BULUNDU (2026-09-06): 100 derece ile
        # arac, onu tamamen kapali bir gecitte (tam ortada duba, dar koridor)
        # ARKAYA yakin bir adayi secip GERI DONDU ve parkuru ters yonde
        # surup duvara carpti. Fan'i one sinirlamak bunu yapisal olarak
        # imkansiz kilar. 70 derece DENENDI ama kose donusunu KORLESTIRDI
        # (sentetik test 9: sagdaki gecit fan disinda kalinca dugum onu
        # 'cikmaz' sanip RAMPA moduna girdi - kosede duvara suruyordu).
        # 85 derece: hem gecit goruluyor hem cos(85)>0 oldugu icin hedefin
        # ileri bileseni HER ZAMAN pozitif - tek bir hedefle arkaya
        # donmek yapisal olarak imkansiz. Parkuru TERS yonde surme riski
        # ayrica referans-yon korumasiyla engelleniyor (bkz. asagisi).
        # KULLANICI KARARI (2026-09-06): "arti eksi 60 derece olsun yani
        # toplam 120 derece". NOT: ±60 simülasyonda 90 derecelik kose
        # donusunu YAPAMADI (yan gecit fan disinda kaliyor, dugum onu
        # 'cikmaz' goruyor) - parkurda keskin donus varsa bu deger
        # yeniden degerlendirilmeli. Duz koridor, slalom ve genis alan
        # senaryolari ±60 ile SORUNSUZ.
        self.declare_parameter('fan_yari_acisi_derece', 60.0)
        # GERI DONUS KORUMASI (kapali dongu simulasyonunda bulundu): saf
        # reaktif bir bosluk-takipcisi, onu kapaninca parkuru TERS yonde
        # surmeye baslayabilir (simulasyonda arac donup geri gitti ve
        # duvara carpti). Gidilen yonun yavas bir ortalamasi (referans
        # yon) tutulur; bu yonden bu esikten fazla sapan adaylar ELENIR.
        # 150 DERECE DENENDI, YETERSIZ (simulasyonda arac yine de adim
        # adim donup parkuru ters yonde surdu): 150'nin altindaki her adim
        # kabul edildigi icin arac birkac tick'te 180'i BULABILIYOR.
        # 110 derece: 90 derecelik kose donusleri hala serbest (referans
        # yon donus suresince zaten pesinden geliyor), ama tam ters yon
        # tek hamlede de kademeli olarak da erisilemiyor.
        self.declare_parameter('geri_donus_esigi_derece', 110.0)
        self.declare_parameter('referans_tau_s', 5.0)
        self.declare_parameter('fan_adimi_derece', 3.0)
        self.declare_parameter('nokta_toplama_yari_acisi_derece', 105.0)
        # KULLANICI KARARI (2026-09-06): "hedefi de 2 metre ileri atsin".
        self.declare_parameter('maks_hedef_mesafesi_m', 2.0)
        self.declare_parameter('min_hedef_mesafesi_m', 1.2)
        self.declare_parameter('durma_payi_m', 0.6)
        # Aracin YANINDAKI/altindaki noktalar yolu kapatmaz - govde 1.65m
        # uzun, base_footprint govdenin ortasinda; bu mesafeden yakin
        # okumalar zaten GECILMIS noktalardir (sentetik testte bulundu:
        # koridorda yan duvar noktalari 'onumu kapatti' sanilip her yon
        # bloke oluyordu).
        self.declare_parameter('min_engel_mesafesi_m', 0.5)
        # Hicbir aday gecmezse guvenlik payi kademeli DUSURULUR (dar
        # slalom gecitlerinde tamamen durmaktansa daralarak gecmek) -
        # kullanilan pay /oto_nokta_durum'da raporlanir.
        # En dusuk kademe SIFIR DEGIL (simulasyonda bulundu): sifir payla
        # planlanan bir gecis, takip hatasi (MPPI duz cizgiyi birebir
        # izlemez, viraji keser) yuzunden GARANTILI surtunmedir. En az
        # %30 pay birakiliyor; asil son-an kacinmasi zaten Nav2 costmap +
        # MPPI katmaninin isi, bu dugum sadece HEDEF secer.
        self.declare_parameter('pay_kademeleri', [1.0, 0.6, 0.3])

        # --- Puanlama agirliklari (0..1 normalize edilmis terimler) ---
        self.declare_parameter('w_aciklik', 1.0)
        self.declare_parameter('w_mesafe', 0.7)
        # 48 KOMBINASYONLUK PARAMETRE TARAMASI ILE SECILDI (2026-09-06):
        # 0.5 -> 0.1. Yuksek duz-gitme tercihi araci koridorun ORTASINDA
        # tutuyor ve onundeki dubaya karar vermeyi GECIKTIRIYORDU; dusuk
        # deger erken taraf secimini serbest birakiyor. Koridor ortalama
        # davranisi bundan ZARAR GORMUYOR cunku ortalamayi asil saglayan
        # sey aciklik terimidir (koridorun ortasi zaten en genis aciklik),
        # bu terim sadece BERABERLIK bozucudur.
        self.declare_parameter('w_duz', 0.1)
        # KAPALI DONGU SIMULASYONUNDA ARTIRILDI (0.35 -> 0.6): tam ortada
        # duba olan dar gecitte dugum sol/sag arasinda KARARSIZ kaldi
        # (ardisik tick'lerde isaret degistirdi), arac ortada kalip
        # manevra alanini tuketti ve dubaya surtundu. Histerezisin
        # artirilmasi "bir tarafa KARAR VER ve o tarafta KAL" davranisini
        # zorluyor - slalomda dogru olan budur.
        self.declare_parameter('w_histerezis', 0.6)
        self.declare_parameter('aciklik_doyum_m', 1.5)
        # Aciklik, hedef NOKTASINDA degil YOL BOYUNCA olculur; hedefi
        # kesen engel de pencereye girsin diye hedeften bu kadar OTESI
        # de dahil edilir (bkz. _yon_degerlendir'in notu).
        self.declare_parameter('aciklik_ileri_pencere_m', 1.5)

        # --- Yayin kararliligi (goal_manager HER yeni hedefte devam eden
        #     manevrayi IPTAL ediyor - bkz. serbest_yon_takipcisi.py notu) ---
        self.declare_parameter('guncelleme_hz', 4.0)
        self.declare_parameter('min_yayin_araligi_s', 0.8)
        self.declare_parameter('hedef_degisim_esigi_m', 0.4)
        self.declare_parameter('hedef_aci_degisim_esigi_derece', 8.0)
        # *** KAPALI DONGU SIMULASYONUNDA BULUNAN KRITIK HATA (2026-09-06) ***
        # Yukaridaki iki esik TEK BASINA kullanilinca dugum "tek atislik
        # planlayici"ya donusuyordu: arac SABIT bir hedefe yaklasirken her
        # tick'te hesaplanan YENI hedef, odom'da neredeyse AYNI noktaya
        # dusuyor (cunku hedef = arac_konumu + kalan_mesafe, arac
        # ilerledikce kalan mesafe kisaliyor) - "degisim kucuk" deyip 11
        # tick ust uste yayin ATLANDI ve arac tam ortadaki dubaya, ancak
        # 1.8m kala tepki vererek girdi. Receding-horizon bir planlayici
        # ilerledikce hedefini MUTLAKA tazelemek zorundadir; asagidaki iki
        # kural bunu garanti eder (esikler yine de goal_manager'in her yeni
        # hedefte manevrayi iptal etmesine karsi makul araliklarda tutuldu).
        self.declare_parameter('maks_hedef_yasi_s', 1.5)
        self.declare_parameter('yenileme_mesafesi_m', 0.7)

        # --- Egim davranislari ---
        self.declare_parameter('egim_esigi_derece', 3.0)   # egim_costmap_ayarlayici.py ile AYNI
        self.declare_parameter('zemin_reddetme_acik', True)
        self.declare_parameter('zemin_reddetme_payi', 0.15)
        self.declare_parameter('egimde_fan_yari_acisi_derece', 20.0)
        self.declare_parameter('egimde_hedef_mesafesi_m', 2.0)
        self.declare_parameter('egimde_on_guvenlik_mesafesi_m', 1.0)

        # --- Rampa/dik engel yaklasimi ---
        self.declare_parameter('rampa_modu_acik', True)
        # 35 -> 25 derece (simulasyonda bulundu): 3m'lik bir koridorda
        # onde duran duz yuzey, base cercevesinde ancak ~±31 derece
        # gorunur; 35 derecelik sektorun kenar kutulari artik YAN
        # DUVARLARI orneklemeye baslayip planarlik testini bozuyordu
        # (gercek rampa REDDEDILIYORDU). 25 derece guvenli sinirin icinde.
        self.declare_parameter('rampa_sektor_yari_acisi_derece', 25.0)
        self.declare_parameter('rampa_kutu_derece', 3.0)
        self.declare_parameter('rampa_duzlem_toleransi_m', 0.25)
        self.declare_parameter('rampa_maks_mesafe_m', 3.0)
        self.declare_parameter('rampa_yaklasma_payi_m', 0.8)
        # Fan'daki en iyi aday, yuzeyden bu kadar OTEYE gecebiliyorsa
        # ortada gercek bir gecis var demektir - RAMPA'ya girilmez
        # (sentetik testte bulundu: kose donuslerinde on duvar da 'duz
        # dik yuzey' olarak olculuyor, ama yandan gecis mevcut).
        self.declare_parameter('rampa_gecis_farki_m', 1.0)
        # *** GUVENLIK (kapali dongu simulasyonunda bulundu) ***: arac dar
        # bir gecitte sikisip YANA dondugunde, karsisina gelen KORIDOR
        # DUVARI da "govdeye dik duz yuzey" testini geciyor ve dugum araci
        # duvara suruyordu. Gercek rampa parkur yonunde, yol boyunca
        # karsilanir - bu yuzden RAMPA modu sadece arac referans yonune
        # (gidilen yonun yavas ortalamasi) YAKINKEN etkinlesir. Yana
        # donmus bir aracin onundeki duvar rampa SAYILMAZ.
        self.declare_parameter('rampa_maks_yon_sapmasi_derece', 45.0)

        # --- Aktivasyon ---
        # OTOMATIK BASLATMA (2026-09-06, kullanici istegi: "otonoma
        # gectigimde nokta atma islemini otomatik olarak baslatacak kodu
        # yaz"): arayuzden OTONOM'a gecilince dugum KENDILIGINDEN aktif
        # olur, MANUEL'e donulunce KENDILIGINDEN durur. /surus_modu,
        # telemetri_sistemi.py::surekli_yayin_dongusu tarafindan HER TURDA
        # tekrar yayinlandigi icin dugum gec baslasa bile modu ogrenir
        # (sadece degisimde yayinlansaydi bu kacirilabilirdi).
        self.declare_parameter('surus_modu_ile_aktiflesir', True)
        # GUVENLIK: /surus_modu bu sure boyunca hic gelmezse (arayuz
        # kapandi/coktu) mod bilgisi BAYAT sayilir ve otomatik aktiflik
        # DUSER - arayuzsuz kalan arac kendi basina hedef uretmeye devam
        # etmesin. Elle acma (/oto_nokta_aktif) bundan etkilenmez.
        self.declare_parameter('surus_modu_bayatlik_s', 5.0)
        self.declare_parameter('otonom_surus_ile_aktiflesir', False)
        self.declare_parameter('veri_bayatlik_esigi_s', 1.0)

        self._p = {}
        self._parametreleri_oku()
        self.add_on_set_parameters_callback(self._parametre_degisti)

        self._aktif_elle = False
        self._aktif_gorev = False
        self._surus_modu = 'MANUEL'   # guvenli varsayilan (surus_koprusu.py ile ayni)
        self._surus_modu_zamani = 0.0
        self._son_aktiflik = False
        self._son_scan = None
        self._son_scan_zamani = 0.0
        self._son_odom = None
        self._son_odom_zamani = 0.0
        self._onceki_theta = 0.0
        self._referans_yon = None  # gidilen yonun yavas ortalamasi (bkz. geri_donus_esigi)
        self._referans_zamani = None
        self._son_hedef_xy = None
        self._son_hedef_yaw = None
        self._son_yayin_zamani = 0.0
        self._son_yayin_konumu = None  # aracin son yayindaki konumu (yenileme_mesafesi icin)
        self._son_mod = 'PASIF'

        self.create_subscription(Bool, '/oto_nokta_aktif', self._elle_aktif_cb, 10)
        self.create_subscription(Bool, '/otonom_surus_aktif', self._gorev_aktif_cb, 10)
        self.create_subscription(String, '/surus_modu', self._surus_modu_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self._goal_pub = self.create_publisher(PoseStamped, '/ugv_goal', 10)
        self._durum_pub = self.create_publisher(String, '/oto_nokta_durum', 10)
        # Rampa/dik engel yaklasimi bayragi. Bu node hedefi yuzeyin KISASINA
        # atar (yuzeye dik hizalanmak icin); yuzeye TIRMANMAK icin costmap'in
        # o yuzeyi engel olarak isaretlemeyi birakmasi gerekir -
        # egim_costmap_ayarlayici.py su an bunu SADECE govde EGILDIKTEN sonra
        # (pitch esigi) yapiyor, yaklasma aninda DEGIL. Bu bayrak o eksigin
        # kapatilmasi icin disari veriliyor (bkz. kullaniciya verilen not).
        self._rampa_pub = self.create_publisher(Bool, '/rampa_yaklasimi', 10)
        self._rampa_bayragi = False

        periyot = 1.0 / max(0.5, float(self._p['guncelleme_hz']))
        self.create_timer(periyot, self._tick)

        self.get_logger().info(
            'on_bosluk_nokta_atici hazir. OTOMATIK: arayuzden OTONOM moda '
            'gecilince kendiliginden baslar, MANUEL moda donulunce durur. '
            'ELLE: ros2 topic pub --once /oto_nokta_aktif '
            'std_msgs/msg/Bool "{data: true}"')

    # ---------------- parametre ----------------
    def _parametreleri_oku(self):
        for ad in self._parametre_adlari():
            self._p[ad] = self.get_parameter(ad).value

    @staticmethod
    def _parametre_adlari():
        return [
            'lidar_yukseklik_m', 'lidar_ileri_ofset_m', 'arac_yari_genislik_m',
            'guvenlik_payi_m', 'fan_yari_acisi_derece', 'fan_adimi_derece',
            'nokta_toplama_yari_acisi_derece',
            'maks_hedef_mesafesi_m', 'min_hedef_mesafesi_m', 'durma_payi_m',
            'min_engel_mesafesi_m', 'pay_kademeleri', 'rampa_gecis_farki_m',
            'geri_donus_esigi_derece', 'referans_tau_s',
            'w_aciklik', 'w_mesafe', 'w_duz', 'w_histerezis', 'aciklik_doyum_m',
            'aciklik_ileri_pencere_m',
            'guncelleme_hz', 'min_yayin_araligi_s', 'hedef_degisim_esigi_m',
            'hedef_aci_degisim_esigi_derece', 'maks_hedef_yasi_s',
            'yenileme_mesafesi_m', 'egim_esigi_derece',
            'zemin_reddetme_acik', 'zemin_reddetme_payi',
            'egimde_fan_yari_acisi_derece', 'egimde_hedef_mesafesi_m',
            'egimde_on_guvenlik_mesafesi_m', 'rampa_modu_acik',
            'rampa_sektor_yari_acisi_derece', 'rampa_kutu_derece',
            'rampa_duzlem_toleransi_m', 'rampa_maks_mesafe_m',
            'rampa_yaklasma_payi_m', 'rampa_maks_yon_sapmasi_derece',
            'surus_modu_ile_aktiflesir', 'surus_modu_bayatlik_s',
            'otonom_surus_ile_aktiflesir',
            'veri_bayatlik_esigi_s',
        ]

    def _parametre_degisti(self, params):
        """Sahada YENIDEN DERLEMEDEN ayar yapabilmek icin (ros2 param set)."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in self._p:
                self._p[p.name] = p.value
                self.get_logger().info(f'parametre guncellendi: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---------------- abonelikler ----------------
    def _elle_aktif_cb(self, msg: Bool):
        yeni = bool(msg.data)
        if yeni != self._aktif_elle:
            self.get_logger().info(f'ELLE aktivasyon: {"ACIK" if yeni else "KAPALI"}')
        self._aktif_elle = yeni

    def _gorev_aktif_cb(self, msg: Bool):
        self._aktif_gorev = bool(msg.data)

    def _surus_modu_cb(self, msg: String):
        yeni = msg.data.strip().upper()
        if yeni in ('MANUEL', 'OTONOM'):
            self._surus_modu = yeni
            self._surus_modu_zamani = time.monotonic()

    def _scan_cb(self, msg: LaserScan):
        self._son_scan = msg
        self._son_scan_zamani = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        self._son_odom = msg
        self._son_odom_zamani = time.monotonic()

    def _aktif_mi(self):
        """Dugum su an hedef uretmeli mi? (uc bagimsiz kaynak)"""
        # 1) Elle acma - saha/tezgah testi icin, moddan BAGIMSIZ.
        if self._aktif_elle:
            return True
        # 2) SURUS MODU = OTONOM (varsayilan yol, kullanici istegi).
        if bool(self._p['surus_modu_ile_aktiflesir']):
            taze = (time.monotonic() - self._surus_modu_zamani) <= float(
                self._p['surus_modu_bayatlik_s'])
            if self._surus_modu == 'OTONOM' and taze:
                return True
        # 3) Gorev sekansi (tabela_etap_yoneticisi) - DOGRULANANA KADAR kapali.
        return bool(self._p['otonom_surus_ile_aktiflesir']) and self._aktif_gorev

    def _aktiflik_degisimini_logla(self):
        """Aktiflik degisimini BIR KEZ loglar (her tick degil) ve donduruur."""
        aktif = self._aktif_mi()
        if aktif == self._son_aktiflik:
            return aktif
        self._son_aktiflik = aktif
        if aktif:
            kaynak = ('elle (/oto_nokta_aktif)' if self._aktif_elle
                      else 'surus modu = ' + self._surus_modu)
            self.get_logger().warn(
                'NOKTA ATMA BASLADI - ' + kaynak + '. Arac ONUNE otomatik '
                'hedef uretilecek (/ugv_goal).')
        else:
            self.get_logger().warn(
                'NOKTA ATMA DURDU (surus modu = ' + self._surus_modu +
                '). Yeni hedef URETILMIYOR.')
        return aktif

    # ---------------- zemin geometrisi ----------------
    def _zemin_mesafeleri(self, acilar, q):
        """Her isinin ZEMINE carpacagi mesafe (m). Zemine hic ulasmayan
        (yukari giden) isinlar icin +inf.

        Isin yonu govde cercevesinde (cos a, sin a, 0); dunya-z bileseni
        donme matrisinin 3. satiriyla bulunur (bkz. o fonksiyonun notu).
        LIDAR zeminden h yukarida oldugundan, dunya-z bileseni uz<0 olan
        bir isin t = h / (-uz) mesafesinde zemin duzlemini keser."""
        r20, r21, _ = _donme_matrisi_ucuncu_satir(q)
        uz = r20 * np.cos(acilar) + r21 * np.sin(acilar)
        h = float(self._p['lidar_yukseklik_m'])
        mesafeler = np.full_like(acilar, np.inf)
        asagi = uz < -1e-4
        mesafeler[asagi] = h / (-uz[asagi])
        return mesafeler

    # ---------------- ana dongu ----------------
    def _tick(self):
        if not self._aktiflik_degisimini_logla():
            self._son_mod = 'PASIF'
            return

        simdi = time.monotonic()
        bayat = float(self._p['veri_bayatlik_esigi_s'])
        if self._son_scan is None or (simdi - self._son_scan_zamani) > bayat:
            self._durum_yayinla({'mod': 'VERI_YOK', 'sebep': 'scan bayat/yok'})
            return
        if self._son_odom is None or (simdi - self._son_odom_zamani) > bayat:
            self._durum_yayinla({'mod': 'VERI_YOK', 'sebep': 'odom bayat/yok'})
            return

        odom = self._son_odom
        q = odom.pose.pose.orientation
        yaw = _yaw_of(q)
        r20, r21, _ = _donme_matrisi_ucuncu_satir(q)
        # Govde ileri ekseninin dunya-z bileseni: + = burun YUKARI (tirmanma)
        egim_on = math.degrees(math.asin(max(-1.0, min(1.0, r20))))
        # Govde sol ekseninin dunya-z bileseni: + = SOL taraf yukarida
        egim_yan = math.degrees(math.asin(max(-1.0, min(1.0, r21))))

        self._referans_yonu_guncelle(yaw, simdi)
        px, py, elenen, ham_sayi = self._engel_noktalari(q)

        egim_esigi = float(self._p['egim_esigi_derece'])
        durum = {
            'referans_yon': round(math.degrees(self._referans_yon), 1),
            'egim_on': round(egim_on, 2),
            'egim_yan': round(egim_yan, 2),
            'nokta': int(px.size),
            'elenen_zemin': int(elenen),
            'ham': int(ham_sayi),
        }

        if egim_on > egim_esigi:
            # (C) TIRMANMA - yanlara GUVENILEMEZ (isinlar engellerin ustunden
            # geciyor, sahte bosluk). Sadece mevcut yonde kisa adim.
            self._egimde_ilerle(px, py, yaw, odom, 'TIRMANMA', durum)
            return
        if egim_on < -egim_esigi:
            # (A) INIS - zemin reddi sahte on-duvari zaten temizledi, ama
            # kalan veri seyrek/gurultulu; yine de yanlara guvenmiyoruz.
            self._egimde_ilerle(px, py, yaw, odom, 'INIS', durum)
            return
        if abs(egim_yan) > egim_esigi:
            # (B) YAN EGIM - yukari kalan yan sahte BOS gorunur. Fan
            # daraltilir, duz gitme tercihi artirilir.
            fan_yari = float(self._p['egimde_fan_yari_acisi_derece'])
            self._normal_ilerle(px, py, yaw, odom, durum, fan_yari_derece=fan_yari,
                                mod_adi='YAN_EGIM')
            return

        self._normal_ilerle(px, py, yaw, odom, durum,
                            fan_yari_derece=float(self._p['fan_yari_acisi_derece']),
                            mod_adi='NORMAL')

    # ---------------- nokta cikarma ----------------
    def _engel_noktalari(self, q):
        """/scan -> base_footprint cercevesinde (x ileri, y sol) GERCEK
        engel noktalari. Zemine carpan isinlar ELENIR (bkz. dosya basi A/B)."""
        scan = self._son_scan
        n = len(scan.ranges)
        r = np.asarray(scan.ranges, dtype=np.float64)
        idx = np.arange(n, dtype=np.float64)
        ham_acilar = scan.angle_min + idx * scan.angle_increment
        # 0 = tam on olacak sekilde kaydir + normalize
        a = ham_acilar - math.pi
        acilar = np.arctan2(np.sin(a), np.cos(a))

        gecerli = np.isfinite(r) & (r > max(scan.range_min, 0.05)) & (r < scan.range_max)
        # NOKTA TOPLAMA acisi, HEDEF ATMA fanindan BAGIMSIZ: kullanici
        # hedefin dar bir on koniye atilmasini istedi (fan_yari_acisi),
        # ama yol acikligini dogru olcmek icin YANLARI da GORMEK gerekir.
        # Ikisi ayni parametreye baglanirsa fan daraltildiginda arac
        # yanindaki engelleri de gormez hale gelirdi.
        gecerli &= np.abs(acilar) <= math.radians(
            float(self._p['nokta_toplama_yari_acisi_derece']))
        ham_sayi = int(np.count_nonzero(gecerli))

        elenen = 0
        if bool(self._p['zemin_reddetme_acik']) and ham_sayi:
            d_zemin = self._zemin_mesafeleri(acilar, q)
            pay = float(self._p['zemin_reddetme_payi'])
            zemin_mi = gecerli & (r >= d_zemin * (1.0 - pay))
            elenen = int(np.count_nonzero(zemin_mi))
            gecerli &= ~zemin_mi

        rr = r[gecerli]
        aa = acilar[gecerli]
        ofset = float(self._p['lidar_ileri_ofset_m'])
        px = rr * np.cos(aa) + ofset
        py = rr * np.sin(aa)
        return px, py, elenen, ham_sayi

    # ---------------- mod: egim (duz ilerle) ----------------
    def _egimde_ilerle(self, px, py, yaw, odom, mod_adi, durum):
        guvenlik = float(self._p['egimde_on_guvenlik_mesafesi_m'])
        yari = float(self._p['arac_yari_genislik_m']) + float(self._p['guvenlik_payi_m'])
        engel_mesafesi = self._yonde_serbest_mesafe(px, py, 0.0, yari)
        durum.update({'mod': mod_adi, 'on_serbest': round(float(engel_mesafesi), 2)})
        if engel_mesafesi < guvenlik:
            durum['sebep'] = 'tam onde engel - hedef gonderilmedi'
            self._durum_yayinla(durum)
            return
        d = min(float(self._p['egimde_hedef_mesafesi_m']),
                max(0.0, engel_mesafesi - float(self._p['durma_payi_m'])))
        if d < float(self._p['min_hedef_mesafesi_m']) * 0.5:
            durum['sebep'] = 'guvenli mesafe cok kisa'
            self._durum_yayinla(durum)
            return
        durum.update({'theta': 0.0, 'mesafe': round(d, 2)})
        self._hedef_gonder(d, 0.0, yaw, odom, durum)

    # ---------------- mod: normal (fan puanlamasi) ----------------
    def _normal_ilerle(self, px, py, yaw, odom, durum, fan_yari_derece, mod_adi):
        maks = float(self._p['maks_hedef_mesafesi_m'])
        min_hedef = float(self._p['min_hedef_mesafesi_m'])
        temel_pay = float(self._p['guvenlik_payi_m'])
        yari_govde = float(self._p['arac_yari_genislik_m'])

        kademeler = list(self._p['pay_kademeleri']) or [1.0]
        en_iyi = None
        kullanilan_pay = None
        for k in kademeler:
            en_iyi = self._en_iyi_aday(px, py, fan_yari_derece,
                                       yari_govde + temel_pay * float(k), yaw)
            if en_iyi is not None:
                kullanilan_pay = round(temel_pay * float(k), 3)
                break

        # RAMPA/DIK ENGEL KARARI: on sektoru kapatan sey govdeye DIK, DUZ
        # bir yuzeyse VE fan'daki en iyi aday o yuzeyin OTESINE gecemiyorsa
        # ortada gercek bir gecis yok demektir. Bu parkurda cikmaz sokak
        # olmadigi icin (bkz. dosya basi (D)) bu, tirmanilacak rampadir.
        yuzey = None
        if bool(self._p['rampa_modu_acik']) and self._yon_referansa_yakin(yaw):
            yuzey = self._rampa_yuzeyi_mi(px, py)
        if yuzey is not None:
            # "Yuzeyin otesine gecilebiliyor mu" sorusu, DAR hedef fanina
            # DEGIL, GENIS ALGI acisina sorulur (bkz. asagidaki fonksiyon).
            # *** SIMULASYONDA BULUNDU (fan ±60'a dusurulunce) ***: kose
            # donusunde yandaki gercek gecit dar fanin DISINDA kaliyor,
            # dugum koseyi 'cikmaz' sanip RAMPA moduna giriyor ve araci
            # duvara suruyordu. Arac o gecide hedef ATMASA bile onu
            # GORUYOR - gormek, "burasi cikmaz degil" demek icin yeterli.
            # Sonuc: kosede RAMPA tetiklenmez; gecerli aday da yoksa
            # KAPALI moduna gecilip DURULUR (duvara surmek yerine).
            # Ayrica KIRPILMIS hedef mesafesi (d) degil GERCEK serbest
            # mesafe kullanilir - hedef 2m'ye kisaltilinca d hicbir zaman
            # 2m'yi asamaz ve test her zaman 'gecemiyor' derdi.
            gecebiliyor = self._genis_acida_gecis_var_mi(px, py, yuzey,
                                                        yari_govde + temel_pay)
            if not gecebiliyor:
                d = max(1.0, yuzey - float(self._p['rampa_yaklasma_payi_m']))
                durum.update({'mod': 'RAMPA_YAKLASMA', 'yuzey_mesafesi': round(yuzey, 2),
                              'theta': 0.0, 'mesafe': round(d, 2), 'pay': kullanilan_pay})
                self._rampa_bayragi_ayarla(True)
                self.get_logger().warn(
                    f'RAMPA/DIK ENGEL: on sektoru govdeye DIK, DUZ bir yuzey '
                    f'kapatiyor ({yuzey:.2f}m) ve fan bunun otesine gecemiyor - '
                    f'yuzeye DIK yaklasiliyor (kapatmak: rampa_modu_acik=False)')
                self._onceki_theta = 0.0
                self._hedef_gonder(d, 0.0, yaw, odom, durum)
                return
        self._rampa_bayragi_ayarla(False)

        if en_iyi is not None:
            _, theta, d, aciklik, _serbest = en_iyi
            self._onceki_theta = theta
            durum.update({'mod': mod_adi, 'theta': round(math.degrees(theta), 1),
                          'mesafe': round(d, 2), 'aciklik': round(aciklik, 2),
                          'pay': kullanilan_pay})
            self._hedef_gonder(d, theta, yaw, odom, durum)
            return

        durum['mod'] = 'KAPALI'
        durum['sebep'] = ('en dusuk guvenlik payinda bile gecerli aday yok ve '
                          'rampa yuzeyi degil - hedef gonderilmedi')
        self._durum_yayinla(durum)

    def _referans_yonu_guncelle(self, yaw, simdi):
        """Gidilen yonun YAVAS ortalamasi (birim vektor uzerinden EMA -
        aci ortalamasi ±pi sinirinda bozulur, vektor ortalamasi bozulmaz)."""
        if self._referans_yon is None:
            self._referans_yon = yaw
            self._referans_zamani = simdi
            return
        # dt olarak OLCULEN duvar saati degil, YAPILANDIRILMIS tick
        # periyodu kullaniliyor: davranis guncelleme_hz'den bagimsiz
        # olarak deterministik olsun (ve testte hizli kosarken referans
        # gercekte oldugundan yavas guncellenmesin).
        dt = 1.0 / max(0.5, float(self._p['guncelleme_hz']))
        self._referans_zamani = simdi
        tau = max(0.5, float(self._p['referans_tau_s']))
        a = 1.0 - math.exp(-dt / tau)
        rx = (1 - a) * math.cos(self._referans_yon) + a * math.cos(yaw)
        ry = (1 - a) * math.sin(self._referans_yon) + a * math.sin(yaw)
        if abs(rx) > 1e-9 or abs(ry) > 1e-9:
            self._referans_yon = math.atan2(ry, rx)

    def _yon_referansa_yakin(self, yaw):
        """Arac hala parkur yonunde mi? (bkz. rampa_maks_yon_sapmasi_derece)"""
        if self._referans_yon is None:
            return True
        fark = math.atan2(math.sin(yaw - self._referans_yon),
                          math.cos(yaw - self._referans_yon))
        return abs(math.degrees(fark)) <= float(self._p['rampa_maks_yon_sapmasi_derece'])

    def _geri_donus_mu(self, dunya_yonu):
        if self._referans_yon is None:
            return False
        fark = math.atan2(math.sin(dunya_yonu - self._referans_yon),
                          math.cos(dunya_yonu - self._referans_yon))
        return abs(math.degrees(fark)) > float(self._p['geri_donus_esigi_derece'])

    def _en_iyi_aday(self, px, py, fan_yari_derece, yari_genislik, yaw):
        """Fan'i tarayip en yuksek puanli adayi dondurur (yoksa None).

        Puan = aciklik + katedilen mesafe - duz-gitme tercihi - histerezis
        (bkz. dosya basi: koridor ortalama / slalom / donus AYNI puanlamadan
        dogal olarak cikar)."""
        maks = float(self._p['maks_hedef_mesafesi_m'])
        min_hedef = float(self._p['min_hedef_mesafesi_m'])
        durma = float(self._p['durma_payi_m'])
        doyum = float(self._p['aciklik_doyum_m'])
        adim = math.radians(max(0.5, float(self._p['fan_adimi_derece'])))
        fan_yari = math.radians(fan_yari_derece)
        w_ac = float(self._p['w_aciklik'])
        w_me = float(self._p['w_mesafe'])
        w_du = float(self._p['w_duz'])
        w_hi = float(self._p['w_histerezis'])

        en_iyi = None
        theta = -fan_yari
        while theta <= fan_yari + 1e-9:
            serbest, _ = self._yon_degerlendir(px, py, theta, yari_genislik)
            d = min(maks, max(0.0, serbest - durma))
            if d >= min_hedef and not self._geri_donus_mu(yaw + theta):
                _, aciklik = self._yon_degerlendir(px, py, theta, yari_genislik, d)
                puan = (w_ac * min(aciklik, doyum) / doyum
                        + w_me * min(d, maks) / maks
                        - w_du * abs(theta) / fan_yari
                        - w_hi * min(1.0, abs(theta - self._onceki_theta) / fan_yari))
                if en_iyi is None or puan > en_iyi[0]:
                    en_iyi = (puan, theta, d, aciklik, serbest)
            theta += adim
        return en_iyi

    def _genis_acida_gecis_var_mi(self, px, py, yuzey, yari_genislik):
        """GENIS algi acisinda, yuzeyin otesine ulasan bir yon var mi?"""
        sinir = math.radians(float(self._p['nokta_toplama_yari_acisi_derece']))
        adim = math.radians(max(1.0, float(self._p['fan_adimi_derece'])))
        esik = yuzey + float(self._p['rampa_gecis_farki_m'])
        theta = -sinir
        while theta <= sinir + 1e-9:
            serbest, _ = self._yon_degerlendir(px, py, theta, yari_genislik,
                                               tavan_ust=esik + 1.0)
            if serbest > esik:
                return True
            theta += adim
        return False

    def _rampa_bayragi_ayarla(self, deger):
        deger = bool(deger)
        if deger != self._rampa_bayragi:
            self._rampa_bayragi = deger
            try:
                self._rampa_pub.publish(Bool(data=deger))
            except Exception:
                pass

    def _yon_degerlendir(self, px, py, theta, yari_genislik, d_icin_aciklik=None,
                         tavan_ust=None):
        """theta yonu icin (serbest_mesafe, yol_acikligi) dondurur.

        serbest_mesafe: arac genisliginde bir koridor supurulunce
        carpilacak ilk engelin mesafesi.

        yol_acikligi: gidilen yol BOYUNCA en dar yanal aciklik.
        *** SIMULASYONDA BULUNAN KRITIK TASARIM HATASI (2026-09-06) ***
        Ilk surumde aciklik SADECE HEDEF NOKTASINDA olculuyordu. Hedef,
        engelin hemen KISASINA atildigi icin "hedef noktasi engelden 1m
        uzakta" gorunuyor ve tam ONDEKI duba ile YANDAN gecen bir yol
        neredeyse AYNI puani aliyordu - arac dubaya 1.4m kalana kadar
        taraf SECMIYOR, sonra 85 derecelik ani bir manevra yapip koridorun
        duvarina donuyordu (kapali dongu testinde bire bir gozlendi).
        Yol boyunca olculen aciklik, tam onde duran bir engeli METRELERCE
        ONCEDEN cezalandirir - arac tarafini erken secer ve yumusak gecer.
        Hedefi KESEN engelin de pencereye girmesi icin hedeften
        aciklik_ileri_pencere_m kadar otesi de dahil edilir."""
        # Normalde serbest mesafeyi hedef ufkunda kirpmak yeterli (daha
        # otesini bilmeye gerek yok). AMA rampa "gecis var mi" kontrolu
        # ufkun COK OTESINI sormak zorunda - orada tavan_ust ile kirpma
        # kaldirilir. (Simulasyonda bulundu: hedef 2m'ye kisaltilinca
        # tavan 2.6m'de kaliyor, rampa esigi 4m hic asilamiyor ve kose
        # her zaman 'rampa' sayiliyordu.)
        tavan = (float(self._p['maks_hedef_mesafesi_m']) + float(self._p['durma_payi_m'])
                 if tavan_ust is None else float(tavan_ust))
        doyum = float(self._p['aciklik_doyum_m'])
        if px.size == 0:
            return tavan, doyum
        ct, st = math.cos(theta), math.sin(theta)
        boyunca = px * ct + py * st
        yanal = -px * st + py * ct
        # Aracin YANINDAN gecmekte olan noktalar (kisa boyunca AMA govde
        # genisliginin DISINDA yanal) yolu kapatmaz - onlar zaten
        # geciliyor. Ama TAM ONDE, govde genisligi ICINDE ve cok yakin bir
        # nokta (ornegin cok yaklasilmis bir duba) MUTLAKA engeldir.
        yanindan_geciyor = (boyunca <= float(self._p['min_engel_mesafesi_m'])) & (
            np.abs(yanal) > float(self._p['arac_yari_genislik_m']))
        onde = (boyunca > 0.0) & (~yanindan_geciyor)
        engelleyen = (np.abs(yanal) <= yari_genislik) & onde
        serbest = tavan if not np.any(engelleyen) else float(
            min(tavan, np.min(boyunca[engelleyen])))

        d = serbest if d_icin_aciklik is None else d_icin_aciklik
        pencere = onde & (boyunca <= d + float(self._p['aciklik_ileri_pencere_m']))
        if not np.any(pencere):
            return serbest, doyum
        return serbest, float(np.min(np.abs(yanal[pencere])))

    def _yonde_serbest_mesafe(self, px, py, theta, yari_genislik):
        return self._yon_degerlendir(px, py, theta, yari_genislik)[0]

    def _rampa_yuzeyi_mi(self, px, py):
        """On sektoru kapatan sey, govdeye DIK ve DUZ, KESINTISIZ bir yuzey mi?

        Rampanin/dik engelin on yuzu duz zeminde GERCEKTEN duvar gibi
        olculur - yanlis olan olcum degil, "cikmaz" cikarimidir (bkz.
        dosya basi (D)).

        KAPALI DONGU SIMULASYONUNDA BULUNDU: ilk surum sektordeki
        x = r*cos(theta) degerlerinin STANDART SAPMASINA bakiyordu ve
        YANLIS POZITIF veriyordu - koridorda onde duba varken (duba
        yakinda, yan duvarlar uzakta) dagilim dar cikabiliyor, dugum
        rampa sanip araci duvara suruyordu. Dogru test PLANARLIK'tir:
        gercek bir dik yuzeyde her acidaki DIK MESAFE (x) AYNIdir ve
        yuzey sektorde KESINTISIZ'dir. Bu yuzden sektor acisal kutulara
        bolunur; HER kutuda nokta bulunmali (kesintisizlik) ve kutularin
        en yakin-x degerleri arasindaki FARK (std degil, tam aralik)
        toleransin altinda kalmalidir.

        Doner: yuzeyin dik mesafesi (m) ya da None."""
        if px.size < 8:
            return None
        sektor = math.radians(float(self._p['rampa_sektor_yari_acisi_derece']))
        acilar = np.arctan2(py, px)
        sec = (np.abs(acilar) <= sektor) & (px > 0.0)
        if int(np.count_nonzero(sec)) < 8:
            return None
        a_sec = acilar[sec]
        x_sec = px[sec]

        kutu = math.radians(max(1.0, float(self._p['rampa_kutu_derece'])))
        kenarlar = np.arange(-sektor, sektor + 1e-9, kutu)
        if kenarlar.size < 4:
            return None
        en_yakinlar = []
        for i in range(kenarlar.size - 1):
            icinde = (a_sec >= kenarlar[i]) & (a_sec < kenarlar[i + 1])
            if not np.any(icinde):
                return None  # KESINTI var -> duz/kesintisiz yuzey degil
            en_yakinlar.append(float(np.min(x_sec[icinde])))
        en_yakinlar = np.asarray(en_yakinlar)
        if float(np.max(en_yakinlar) - np.min(en_yakinlar)) > float(
                self._p['rampa_duzlem_toleransi_m']):
            return None  # dik mesafe sabit degil -> duz dik yuzey degil
        ortalama = float(np.mean(en_yakinlar))
        if ortalama > float(self._p['rampa_maks_mesafe_m']):
            return None
        return ortalama

    # ---------------- hedef yayini ----------------
    def _hedef_gonder(self, d, theta, yaw, odom, durum):
        """Aday (mesafe, aci) -> odom cercevesinde PoseStamped.

        goal_manager_node HER yeni /ugv_goal'da devam eden Nav2 manevrasini
        IPTAL EDIYOR (bkz. serbest_yon_takipcisi.py'nin ayni notu) - bu
        yuzden kucuk degisikliklerde YENIDEN GONDERILMEZ, yoksa arac
        surekli iptal edilip hic ilerleyemez ("titreme")."""
        hedef_yaw = math.atan2(math.sin(yaw + theta), math.cos(yaw + theta))
        ox = odom.pose.pose.position.x
        oy = odom.pose.pose.position.y
        gx = ox + d * math.cos(hedef_yaw)
        gy = oy + d * math.sin(hedef_yaw)

        simdi = time.monotonic()
        if self._son_hedef_xy is not None:
            dx = gx - self._son_hedef_xy[0]
            dy = gy - self._son_hedef_xy[1]
            aci_farki = abs(math.atan2(math.sin(hedef_yaw - self._son_hedef_yaw),
                                       math.cos(hedef_yaw - self._son_hedef_yaw)))
            kucuk_degisim = (
                math.hypot(dx, dy) < float(self._p['hedef_degisim_esigi_m'])
                and math.degrees(aci_farki) < float(self._p['hedef_aci_degisim_esigi_derece'])
            )
            # RECEDING HORIZON ZORUNLULUGU (bkz. maks_hedef_yasi_s notu):
            # hedef eskidiyse ya da arac belirgin yol katettiyse "degisim
            # kucuk" olsa BILE tazelenir - yoksa dugum tek atislik olur.
            yasli = (simdi - self._son_yayin_zamani) > float(self._p['maks_hedef_yasi_s'])
            yol_aldi = False
            if self._son_yayin_konumu is not None:
                yol_aldi = math.hypot(ox - self._son_yayin_konumu[0],
                                      oy - self._son_yayin_konumu[1]) > float(
                    self._p['yenileme_mesafesi_m'])
            if yasli or yol_aldi:
                kucuk_degisim = False
            cok_erken = (simdi - self._son_yayin_zamani) < float(self._p['min_yayin_araligi_s'])
            if kucuk_degisim or cok_erken:
                durum['yayin'] = 'atlandi' + ('/kucuk' if kucuk_degisim else '/erken')
                self._durum_yayinla(durum)
                return

        hedef = PoseStamped()
        hedef.header.stamp = self.get_clock().now().to_msg()
        hedef.header.frame_id = 'odom'
        hedef.pose.position.x = float(gx)
        hedef.pose.position.y = float(gy)
        hedef.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(hedef_yaw)
        hedef.pose.orientation.x = qx
        hedef.pose.orientation.y = qy
        hedef.pose.orientation.z = qz
        hedef.pose.orientation.w = qw
        self._goal_pub.publish(hedef)

        self._son_hedef_xy = (gx, gy)
        self._son_hedef_yaw = hedef_yaw
        self._son_yayin_zamani = simdi
        self._son_yayin_konumu = (ox, oy)
        durum['yayin'] = 'gonderildi'
        durum['hedef_odom'] = [round(gx, 2), round(gy, 2)]
        self._durum_yayinla(durum)

    def _durum_yayinla(self, durum):
        mod = durum.get('mod', '?')
        if mod != self._son_mod:
            self.get_logger().info(f'MOD: {self._son_mod} -> {mod} ({durum})')
            self._son_mod = mod
        try:
            self._durum_pub.publish(String(data=json.dumps(durum)))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OnBoslukNoktaAtici()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
