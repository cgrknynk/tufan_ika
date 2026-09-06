#!/usr/bin/env python3
"""Haritasiz/bilinmeyen parkurda (S donusleri dahil) otonom ilerleme -
2026-09-01, kullanici istegi ("silah iptal, '1' tabelasinda ileri baslasin,
S donuslerinde bariyerin arkasina hedef atmasin").

ONCEKI DENENEN YAKLASIM (REDDEDILDI, uygulanmadi): aracin MEVCUT sabit
yonunde surekli kisa hedef gondermek. Kullanicinin kendi tespiti: S
donuslerinde bir bariyer varsa, sabit-yon hedefi bariyerin ARKASINDA/
ICINDE kalabilir - NavigateToPose oraya planlama yapamaz, Nav2 recovery
davranislarina (donme/geri gitme) girer ("bariyere gelir ve kilitlenir").

BU DOSYANIN YAKLASIMI (gap-following / "en genis acik yone git"):
sabit bir yon varsaymak yerine, HER TICK'TE filtrelenmis LIDAR taramasina
(/scan, scan_front_filter.py'nin 150 derece on-koni ciktisi) bakip
GERCEKTEN ACIK olan en genis surekli aci araligini bulur, o yonde KISA
(birkac metre) bir hedef gonderir. Bariyer hangi yondeyse o yon zaten
"kapali" sayilir (ACIK_MESAFE_ESIGI_M altinda kalir) - hedef ASLA bir
bariyerin arkasina dusmez, cunku secim dogrudan o anki gercek bos
alana gore yapilir. Periyodik tekrar (GUNCELLEME_PERIYODU_S) ile arac
S donuslerini REAKTIF olarak takip eder.

Acik-koni acisi HAM SCAN ACISINDA ±180 derece civarindadir (scan_front_
filter.py'nin notu: "TF: base_footprint -> lidar_link su an net 180
derece donuk", yani GERCEK ON ham aci ±180'e yakin, 0 degil). Bu yuzden
_on_merkezli() ile ham aciyi pi kadar kaydirip normalize ediyoruz -
donusumden sonra 0 = tam on, +/- degerler sag/sol.

AKTIVASYON: /otonom_surus_aktif (Bool) - tabela_etap_yoneticisi.py
tarafindan yonetilir ('1' tabelasinda True, Stop tabelasinda 15sn
icin False, sonra tekrar True). Bu node PASIF iken hicbir hedef
gondermez - goal_manager_node/Nav2 baska bir kaynaktan (orn. rampa,
gecici_dur) gelen hedefleri BOZMAZ.

GUVENLIK: hicbir yeterince genis (>=MIN_GAP_GENISLIGI_DERECE) acik gap
bulunamazsa (on taraf tamamen kapali) HICBIR HEDEF GONDERILMEZ - arac
mevcut hedefini tamamlayip durur, yanlis yone hedef atilmaz.

EGIM/RAMPA ISTISNASI (2026-09-01, CANLI TESTTE BULUNDU): dik engelden
(rampa) gecerken arac SOL taraftaki bariyerin USTUNE cikti - LIDAR bu
sirada (govde egildigi icin tarama duzlemi de egiliyor - bkz.
egim_costmap_ayarlayici.py'nin ayni fiziksel nedeni) sol tarafi YANLIŞLIKLA
"acik" gordu (isin bariyerin USTUNDEN gecip cok daha uzaktaki bosluga
carpmasi ihtimali) - gap-following bu yanlis "acik" yonu secip araci DAHA
DA sola yonlendirdi, bariyere tirmanma kotulesti (kendi kendini besleyen
bir dongu). 2D LIDAR TEK duzlemde tarar - govde egimliyken yan taraftaki
ALCAK bir engelin USTUNDEN gecip GORMEMESI bilinen fiziksel bir sinirdir,
yazilimla "duzeltilemez".

COZUM: |pitch| esigi (egim_costmap_ayarlayici.py ile AYNI deger, iki
sistem TUTARLI calissin) asilinca gap-following DEVRE DISI - LIDAR'in yan
taraflara guvenilmez oldugu bu durumda sadece MEVCUT yonde duz gidilir
(sadece TAM ONDEKI dar bir aciya bakip asgari bir guvenlik kontrolu
yapilir). Ayrica IMU'nun ROLL'u (LIDAR'DAN TAMAMEN BAGIMSIZ bir sinyal -
aracin bir tarafa yaslanmasi/tirmanmasi dogrudan olculur) izlenir: roll
esigi asilirsa KUCUK, SINIRLI bir yon duzeltmesi (yaslanma yonunun TERSINE)
uygulanir. *** ROLL_DUZELTME_YONU CANLI TESTTE DOGRULANMALI - ters cikarsa
tek satirlik degeri -1.0 yap (bkz. asagida, ayni desen konum_birlestirici.
py'nin IMU montaj duzeltmesinde ve arduino_motor_kontrol.py'nin PID
yon carpaninda kullanildi). ***

U DONUSU ISTISNASI (2026-09-01, IKI KEZ CANLI TESTTE REVIZE EDILDI):

  ILK DENEME (REDDEDILDI): "U donusunu /scan_raw'daki (TAM 360 derece ham
  tarama) TAM ONDE bir duvar tespit edilince, YINE /scan_raw'da MEVCUT
  YONE bakmadan EN UZAGA ulasan aciklik" olarak coz. KULLANICI DUZELTTI:
  LIDAR'in govde/baski nedeniyle ~200 derecelik ARKA kismi FIZIKSEL
  OLARAK KAPALI - /scan_raw "360 derece" gibi gorunse de verinin
  yarisindan fazlasi kullanilamaz baski-yansimasi gurultusudur, GERCEK
  ORTAM VERISI DEGIL. Bu yuzden "ham taramada genis aci ara" fikri ya
  hicbir sey bulamiyordu (zararsiz ama islevsiz) ya da KOTU ihtimalle
  baski bolgesindeki gurultulu bir okumayi "uzak acik yon" sanip TEHLIKELI
  bir hedef uretebilirdi. Sensorun goremedigi yone (mevcut yondan >~75
  derece) HICBIR yazilim hedef hesaplayamaz - bu donanimsal bir sinir.

  GUNCEL YAKLASIM (HIBRIT - odometri + LIDAR birlikte, kullanici istegi:
  "lidardan da destek alsin"): U donusunun HANGI YONE oldugunu SADECE
  LIDAR'A SORMAK YETERSIZ (LIDAR o yonu, arac donmeden, zaten goremiyor).
  Bu yuzden IKI BAGIMSIZ SINYAL BIRLIKTE degerlendirilir:
    1) ODOMETRI (yaw egilimi, GPS/IMU tabanli - LIDAR montaj kisitindan
       TAMAMEN BAGIMSIZ): duvar/cikmaz tespit edilmeden onceki birkac
       saniyede arac zaten sola mi saga mi kivriliyordu? Yol boyle
       kivriliyorsa U donusu de BUYUK IHTIMALLE ayni yone devam eder
       (bir insanin kor bir virajda yolun kivrildigi yone devam etmesi
       gibi bir cikarim - tahmin degil, GERCEK harekete dayali).
    2) LIDAR (gorunur koninin sol/sag kenar bolgelerindeki ortalama acik
       mesafe farki - SADECE guvenilir/temiz /scan icinde, /scan_raw
       ARTIK KULLANILMIYOR): o an FIILEN GORULEBILEN kismi bilgi.
  Ikisi AYNI yonu gosteriyorsa YUKSEK GUVENLE o yon secilir. CELISIRLERSE
  ya da odometri egilimi zayifsa (esik altinda), LIDAR'IN DOGRUDAN
  GOZLEMLENEBILIR kenar-aciklik farki TERCIH EDILIR - bir egilim
  TAHMININDEN daha guvenilir bir GERCEK GOZLEM oldugu icin (bkz.
  _udonus_yon_karari_ver).

  Yon KARARI verilince (SOL ya da SAG), bu karara SADIK KALINIR (KILIT) -
  her tick yeniden karar VERILMEZ (onceki salinim/kararsizlik sorununun
  asil nedeni buydu). Kilitli yonde, SADECE guvenilir/temiz /scan icinde,
  gorunur koninin o yondeki KENARINA en yakin gecerli-genislikte bosluk
  secilir - arac o yone dogru adim adim doner, HER TICK'TE tekrar taranan
  /scan onune yeni bir aciklik getirince (duvar artik tespit edilmeyince)
  kilit acilip normal moda donulur.

  *** BILINEN SINIRLAMALAR / CANLI TESTTE IZLENMESI GEREKENLER ***
  (bkz. kullaniciya verilen analiz, 2026-09-01):
  - Odometri egilimi/LIDAR kenar-farki YANLIS yon isaret ederse (orn.
    corridor'un hemen onceki kivrimi gercek U donusu yonunun TERSIYSE),
    sistem yine de o yone KILITLENIR - 20s zaman asimina kadar BASKA
    yonu DENEMEZ (şu an OTOMATIK yon-degistirme YOK).
  - ON_DUVAR_MESAFE_ESIGI_M (2.5m) tetigi, GERCEK bir U donusu YERINE
    sadece yakin bir duba/engel yuzunden de tetiklenebilir (yanlis
    pozitif) - bu durumda gereksiz yere "kenar-kilitleme" moduna girer.
  - UDONUS_HEDEF_ACISI_DERECE (65 derece), guvenli LIDAR sinirina (~75
    derece) yakin - kenar bolgesinde veri daha seyrek/gurultulu olabilir.
  - Bu node'un hedef koydugu yer ile Nav2/MPPI'nin GERCEKTE o yola nasil
    gittigi ayri katmanlar - cok DAR bir donusde MPPI k-nokta manevra
    gerektirebilir, bu node bunu ORKESTRE ETMEZ, sadece hedef verir.
  - YAW_EGILIM_ESIGI_DERECE/YAW_GECMISI_SANIYE degerleri TAHMINI,
    CANLI TESTTE dogrulanmali.
"""
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


ACIK_MESAFE_ESIGI_M = 1.5       # bu mesafeden UZAK noktalar "acik" sayilir
MIN_GAP_GENISLIGI_DERECE = 20.0  # bu genislikten DAR gap'ler guvenilir sayilmaz
MIN_HEDEF_MESAFESI_M = 0.8
GUNCELLEME_PERIYODU_S = 1.5

# NORMAL MOD (bkz. _tick) - KESIN COZUM v2 (2026-09-01, kullanici istegi:
# "rastgele ileri nokta atmaktansa sag/sol bariyerlerin orta noktasina
# atsin ve gorebildigi en uc noktaya atsin, onde engel yoksa 10 metre
# ileri atsin"): oncesinde hedef_mesafe SABIT bir oran (%75, en fazla
# 5m) ile hesaplaniyordu - acik bir koridorda bile gereksiz kisa
# kaliyordu. Artik IKI ACIK DURUM var:
#   1) ONDE (secilen gap icinde) ACIK_ONDE_ENGEL_YOK_ESIGI_M'den UZAKTA
#      hicbir engel yoksa - "onde engel yok" - UZAK_HEDEF_MESAFESI_M
#      (10m) sabit hedef atilir (global_costmap genisligi bunu
#      guvenle karsilayacak sekilde 22m'ye cikarildi, bkz. nav2_params.yaml).
#   2) Engel gorunuyorsa (gap'i sinirlayan bariyer/duba) - hedef
#      GORULEN SINIRA (acik_mesafe) NORMAL_GUVENLIK_PAYI_M kadar KISA
#      atilir (once %75 ile agresif kesiliyordu, simdi neredeyse
#      gorulebilen en uc noktaya kadar gidiliyor).
# Aci (on_merkezli_aci), _en_genis_gap'in zaten hedef_aci=0'a (mevcut
# yon) en yakin gap'i secmesiyle DOGAL olarak o gap'i sinirlayan sag/sol
# bariyerlerin ACISAL ORTA NOKTASINI verir (gap = iki bariyer arasi acik
# bant, orta_aci = bu bandin merkezi) - ayri bir hesap GEREKMEDI.
ACIK_ONDE_ENGEL_YOK_ESIGI_M = 6.0  # gap'teki en yakin nokta bundan UZAKSA "onde engel yok"
UZAK_HEDEF_MESAFESI_M = 10.0        # onde engel yoksa bu kadar ileri hedef atilir
NORMAL_GUVENLIK_PAYI_M = 0.5        # engel gorunuyorsa, gorulen sinirin bu kadar KISASINA hedef atilir

# U DONUSU MODU (bkz. _udonus_isle) - KESIN COZUM v2 (2026-09-01,
# kullanici istegi: "u donusu algilanirsa onundeki engele 1.5 metre
# yakinina atsin ama yonelimi engele PARALEL atsin - o zaman engelin
# karsisinda durup kilitlenmez veya geri donmez"): oncesinde hedefin
# ORIENTATION'i (goal.pose.orientation) hep hedef POZISYONUNA dogru
# bakacak sekilde ayarlaniyordu - dar bir U donusunde bu, aracin neredeyse
# DUVARA DONUK bir son poza ulasmaya calismasi anlamina geliyordu (MPPI
# icin zor/imkansiz bir hedef - "titreme"/kilitlenme/geri gitme egilimi
# buradan da geliyordu, HEDEF_ACI_DEGISIM_ESIGI_DERECE duzeltmesinden
# BAGIMSIZ ayri bir neden). Artik pozisyon HALA kilitli-yon kenarina
# yakin bir noktaya (UDONUS_HEDEF_MESAFESI_M~1.5m) atiliyor AMA
# ORIENTATION ayri hesaplaniyor: mevcut_yaw +/- 90 derece (kilitli yone
# dogru) - yani "engele PARALEL" (engel govdeye ~dik oldugu icin, ona
# paralel olmak mevcut yondan 90 derece donmek demektir). Bu sayede
# Nav2'nin ulasmasi gereken son poz "duvara bakan" degil "duvar boyunca
# donmeye hazir" oluyor - dogal bir ileri yay ile ulasilabilir.
UDONUS_HEDEF_MESAFESI_M = 1.5
UDONUS_GUVENLIK_PAYI_M = 0.3
# CANLI TESTTE BULUNDU (2026-09-01): bkz. _hedef_gonder'in ustundeki notu -
# goal_manager_node HER yeni /ugv_goal'da devam eden manevrayi IPTAL
# EDIYOR, bu esik altindaki kucuk degisiklikler icin YENIDEN GONDERME
# yapilmayarak arac gercek bir harekete baslamaya FIRSAT buluyor.
HEDEF_DEGISIM_ESIGI_M = 0.4
# KESIN COZUM (2026-09-01, CANLI TESTTE BULUNDU - "rota atiyor fakat
# ilerleyemiyor, titriyor, bir ileri bir geri yapiyor"): NORMAL_MAX_
# HEDEF_MESAFESI_M'nin 2.5m'den 5.0m'ye cikarilmasi, YUKARIDAKI (sadece
# XY mesafesi bakan) esigi ISE YARAMAZ hale getirdi - gap-secimindeki
# birkac derecelik LIDAR gurultusu (komsu iki aday gap'in genislik/
# merkez hesabi ucundan ucuna gecmesi gibi kucuk nedenlerle) artik DAHA
# UZUN bir hedef_mesafe ile CARPILINCA (aci_farki * mesafe) 0.4m esigini
# kolayca ASIYOR - HEDEF_DEGISIM_ESIGI_M koruma tekrar devre disi kalip
# goal_manager'in kendini iptal etme dongusu (bkz. _hedef_gonder notu)
# GERI GELIYORDU. Cozum: republish'i SADECE XY mesafesine degil, ACI
# FARKINA da bagla - aci kararliligi mesafeden BAGIMSIZ gercek bir
# olculer, boylece uzak hedeflerde de ayni gurultu toleransi korunur.
HEDEF_ACI_DEGISIM_ESIGI_DERECE = 8.0

# --- EGIM/RAMPA ISTISNASI (bkz. dosya basi notu) ---
PITCH_ESIGI_DERECE = 3.0  # egim_costmap_ayarlayici.py ile AYNI (2026-09-01 dogrulandi)
DUZ_GIDIS_MESAFE_M = 2.0  # egimde gap-following yerine bu kadar duz ileri
DUZ_GIDIS_ON_ACI_YARISI_DERECE = 10.0  # "tam on" guvenlik kontrolu icin dar pencere
DUZ_GIDIS_ON_MESAFE_ESIGI_M = 1.0  # bu altinda tam onde engel varsa hedef gonderilmez
ROLL_DUZELTME_ESIGI_DERECE = 3.0  # bu ustunde roll = "bir tarafa yaslaniyor/tirmaniyor"
ROLL_DUZELTME_KAZANC = 2.0  # duzeltme_rad = KAZANC * roll_rad (asagidaki maksla sinirli)
ROLL_DUZELTME_MAKS_DERECE = 25.0
ROLL_DUZELTME_YONU = 1.0  # *** CANLI TESTTE TERS CIKARSA -1.0 YAP (bkz. dosya basi notu) ***

# --- U DONUSU ISTISNASI (bkz. dosya basi notu - odometri-tabanli yon karari) ---
ON_DUVAR_YARI_ACISI_DERECE = 30.0  # "tam on" - bu pencerede duvar/cikmaz aranir
ON_DUVAR_MESAFE_ESIGI_M = 2.5  # bu altinda min mesafe varsa "U donusu/cikmaz yaklasti"
YAW_GECMISI_SANIYE = 4.0  # yon karari icin bu kadar geriye bakip egilim hesaplanir
YAW_EGILIM_ESIGI_DERECE = 5.0  # bu altinda "belirgin egilim yok" sayilir (kenar-acikligi tercih edilir)
KENAR_IC_SINIRI_DERECE = 40.0  # "hangi kenar daha acik" karsilastirmasi bu acinin OTESINDEKI noktalarla yapilir
UDONUS_HEDEF_ACISI_DERECE = 65.0  # kilitli yonde, guvenli sinirin (±75) icinde kalan kenar-yakini hedef acisi
UDONUS_ZAMAN_ASIMI_S = 20.0  # bu sureden fazla kilitli kalirsa hedef gonderilmez, hata loglanir


def _on_merkezli(ham_aci_rad):
    """Ham scan acisini (radyan, TF geregi ±180 civari = ON) 0=tam-on
    olacak sekilde kaydirir ve -pi..pi'ye normalize eder."""
    a = ham_aci_rad - math.pi
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pitch_of(q):
    """egim_costmap_ayarlayici.py'nin _pitch_of'uyla AYNI formul (tutarlilik)."""
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def _roll_of(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.atan2(sinr_cosp, cosr_cosp)


class SerbestYonTakipcisi(Node):
    def __init__(self):
        super().__init__('serbest_yon_takipcisi')

        self._aktif = False
        self._son_scan = None
        self._son_odom = None
        self._yaw_gecmisi = deque()  # (zaman_monotonic, yaw_rad)

        self._udonus_kilitli_yon = None  # None, +1.0 (sol), -1.0 (sag)
        self._udonus_kilit_zamani = None
        self._son_gonderilen_hedef = None  # (x, y) odom cercevesinde - bkz. _hedef_gonder
        self._son_gonderilen_hedef_aci = 0.0  # radyan, odom cercevesinde - bkz. HEDEF_ACI_DEGISIM_ESIGI_DERECE

        self.create_subscription(Bool, '/otonom_surus_aktif', self._aktif_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/ugv_goal', 10)

        self.create_timer(GUNCELLEME_PERIYODU_S, self._tick)

        self.get_logger().info(
            'serbest_yon_takipcisi hazir (PASIF) - /otonom_surus_aktif=True '
            'gelince LIDAR\'a gore en acik yone dogru kisa-mesafeli hedefler '
            'gondermeye baslar.'
        )

    def _aktif_cb(self, msg):
        yeni = bool(msg.data)
        if yeni != self._aktif:
            self.get_logger().info(f"🧭 Serbest yön takibi: {'AKTİF' if yeni else 'PASİF'}")
        self._aktif = yeni

    def _scan_cb(self, msg):
        self._son_scan = msg

    def _odom_cb(self, msg):
        self._son_odom = msg
        yaw = _yaw_of(msg.pose.pose.orientation)
        simdi = time.monotonic()
        self._yaw_gecmisi.append((simdi, yaw))
        while self._yaw_gecmisi and simdi - self._yaw_gecmisi[0][0] > YAW_GECMISI_SANIYE:
            self._yaw_gecmisi.popleft()

    def _en_genis_gap(self, hedef_aci=0.0):
        """(on_merkezli_aci, o yondeki_acik_mesafe) veya None doner.
        hedef_aci: adaylar arasindan BUNA en yakin olan tercih edilir
        (varsayilan 0.0 = tam on/mevcut yon; U donusu kilitli-yon modunda
        kenar acisi verilir - bkz. _udonus_hedefe_git)."""
        msg = self._son_scan
        n = len(msg.ranges)
        acik = []  # (on_merkezli_aci, mesafe) - sadece ACIK_MESAFE_ESIGI_M ustundekiler
        for i in range(n):
            ham_aci = msg.angle_min + i * msg.angle_increment
            r = msg.ranges[i]
            if not math.isfinite(r) or r < ACIK_MESAFE_ESIGI_M:
                continue
            acik.append((_on_merkezli(ham_aci), r))

        if not acik:
            return None

        acik.sort(key=lambda t: t[0])

        # Ardisik acik noktalari (aci farki normal ornekleme araligindan
        # buyukse "kopuk" sayilir) gruplara ayir.
        aci_artis = abs(msg.angle_increment)
        bosluk_esigi = max(aci_artis * 3.0, math.radians(2.0))
        gruplar = []
        mevcut = [acik[0]]
        for k in range(1, len(acik)):
            if acik[k][0] - acik[k - 1][0] <= bosluk_esigi:
                mevcut.append(acik[k])
            else:
                gruplar.append(mevcut)
                mevcut = [acik[k]]
        gruplar.append(mevcut)

        adaylar = []
        for g in gruplar:
            genislik_derece = math.degrees(g[-1][0] - g[0][0])
            if genislik_derece < MIN_GAP_GENISLIGI_DERECE:
                continue
            orta_aci = (g[0][0] + g[-1][0]) / 2.0
            min_mesafe = min(r for _, r in g)
            adaylar.append((orta_aci, min_mesafe))

        if not adaylar:
            return None

        # hedef_aci'ye EN YAKIN olani tercih et (normal modda 0=mevcut
        # yon -> gereksiz sag-sol savrulmayi azaltir; U donusu modunda
        # kilitli-yon kenari -> o yone dogru adim adim doner).
        adaylar.sort(key=lambda t: abs(t[0] - hedef_aci))
        return adaylar[0]

    def _on_duvar_var_mi(self):
        """TAM ONDE (dar pencere), GUVENILIR /scan'e gore duvar/cikmaz var
        mi - U donusu/keskin donus yaklastigini anlamak icin. SADECE
        /scan (guvenli, temiz 150 derece koni) kullanir."""
        msg = self._son_scan
        n = len(msg.ranges)
        esik = math.radians(ON_DUVAR_YARI_ACISI_DERECE)
        mesafeler = []
        for i in range(n):
            ham_aci = msg.angle_min + i * msg.angle_increment
            if abs(_on_merkezli(ham_aci)) > esik:
                continue
            r = msg.ranges[i]
            if math.isfinite(r):
                mesafeler.append(r)
        if not mesafeler:
            return False  # veri yok - varsayilan: duvar YOK
        return min(mesafeler) < ON_DUVAR_MESAFE_ESIGI_M

    def _yaw_egilimi(self):
        """Son YAW_GECMISI_SANIYE icindeki en eski/en yeni yaw farkini
        (normalize, radyan) doner - pozitif: sola kivriliyordu (CCW),
        negatif: saga. Yeterli gecmis yoksa 0.0."""
        if len(self._yaw_gecmisi) < 2:
            return 0.0
        _, ilk_yaw = self._yaw_gecmisi[0]
        _, son_yaw = self._yaw_gecmisi[-1]
        fark = son_yaw - ilk_yaw
        return math.atan2(math.sin(fark), math.cos(fark))

    def _daha_acik_kenar(self):
        """Belirgin bir yaw egilimi yokken, GUVENILIR /scan'in gorunur
        koninin sol/sag kenar bolgelerindeki (KENAR_IC_SINIRI_DERECE
        otesi) ortalama acik mesafeyi karsilastirir. +1.0=sol, -1.0=sag."""
        msg = self._son_scan
        n = len(msg.ranges)
        ic_sinir = math.radians(KENAR_IC_SINIRI_DERECE)
        sol_mesafeler = []
        sag_mesafeler = []
        for i in range(n):
            ham_aci = msg.angle_min + i * msg.angle_increment
            a = _on_merkezli(ham_aci)
            r = msg.ranges[i]
            if not math.isfinite(r):
                continue
            if a > ic_sinir:
                sol_mesafeler.append(r)
            elif a < -ic_sinir:
                sag_mesafeler.append(r)
        sol_ort = sum(sol_mesafeler) / len(sol_mesafeler) if sol_mesafeler else 0.0
        sag_ort = sum(sag_mesafeler) / len(sag_mesafeler) if sag_mesafeler else 0.0
        return 1.0 if sol_ort >= sag_ort else -1.0

    def _udonus_yon_karari_ver(self):
        """Kilitli yon yoksa, ODOMETRI (yaw egilimi) VE LIDAR (gorunur
        kenar aciklik farki) BIRLIKTE degerlendirilir (kullanici istegi:
        "lidardan da destek alsin"). Ikisi AYNI yonu gosteriyorsa yuksek
        guvenle o yon secilir. CELISIRLERSE ya da odometri egilimi zayifsa
        (esik altinda), LIDAR'in DOGRUDAN GOZLEMLENEBILIR kenar-aciklik
        farki tercih edilir - bir egilim TAHMININDEN daha guvenilir bir
        GERCEK GOZLEM oldugu icin. Sonuc: +1.0=sol, -1.0=sag."""
        egilim = self._yaw_egilimi()
        esik = math.radians(YAW_EGILIM_ESIGI_DERECE)
        odom_yonu = None
        if egilim > esik:
            odom_yonu = 1.0
        elif egilim < -esik:
            odom_yonu = -1.0

        lidar_yonu = self._daha_acik_kenar()

        if odom_yonu is None:
            self.get_logger().info(
                f"↩️ Yön kararı: belirgin odometri eğilimi yok - LIDAR kenar "
                f"açıklığına göre {'SOL' if lidar_yonu > 0 else 'SAĞ'}.")
            return lidar_yonu

        if odom_yonu == lidar_yonu:
            self.get_logger().info(
                f"↩️ Yön kararı: odometri VE LIDAR aynı yönü gösteriyor "
                f"({'SOL' if odom_yonu > 0 else 'SAĞ'}) - yüksek güven.")
            return odom_yonu

        self.get_logger().warn(
            f"⚠️ Yön kararı: odometri ({'SOL' if odom_yonu > 0 else 'SAĞ'}) ile LIDAR "
            f"({'SOL' if lidar_yonu > 0 else 'SAĞ'}) ÇELİŞİYOR - doğrudan gözlem "
            "olan LIDAR tercih edildi.")
        return lidar_yonu

    def _on_mesafe(self):
        """Tam on'a (on_merkezli_aci≈0) yakin dar bir pencerede en yakin
        engel mesafesini doner (yoksa None) - egim sirasinda blindly ileri
        gitmemek icin asgari bir guvenlik kontrolu."""
        msg = self._son_scan
        n = len(msg.ranges)
        esik = math.radians(DUZ_GIDIS_ON_ACI_YARISI_DERECE)
        mesafeler = []
        for i in range(n):
            ham_aci = msg.angle_min + i * msg.angle_increment
            if abs(_on_merkezli(ham_aci)) <= esik:
                r = msg.ranges[i]
                if math.isfinite(r):
                    mesafeler.append(r)
        return min(mesafeler) if mesafeler else None

    def _hedef_gonder(self, hedef_yaw_odom, hedef_mesafe, hedef_yon_yaw_odom=None):
        """hedef_yon_yaw_odom verilmezse (varsayilan davranis) hedef POZISYONA
        dogru bakilir - eskisi gibi. VERILIRSE (bkz. _udonus_isle -
        "engele PARALEL" orientation), pozisyon YINE hedef_yaw_odom/
        hedef_mesafe'den hesaplanir AMA goal'un ORIENTATION'i bu AYRI
        acidan gelir - dar U donuslerinde "duvara bakan" degil "duvar
        boyunca donmeye hazir" bir son poz hedeflemek icin (bkz.
        UDONUS_HEDEF_MESAFESI_M ustundeki dosya basi notu).

        CANLI TESTTE BULUNDU (2026-09-01) - ASIL "araç hiç hareket
        etmiyor" SORUNUYDU: goal_manager_node, her YENI /ugv_goal
        gelisinde ONCEKI NavigateToPose istegini goal_handle.
        cancel_goal_async() ile IPTAL EDIP sifirdan basliyor (bkz. o
        dosyanin _on_new_goal'i). GUNCELLEME_PERIYODU_S (1.5s) her
        seferinde YENI bir hedef gonderdigi icin, arac bir hedefe dogru
        GERCEKTEN hareket etmeye baslayamadan (MPPI/Nav2'nin bir yol
        planlayip yurutmesi saniyeler alir) surekli IPTAL EDILIYORDU -
        motor komutu HEP notr (1500,1500) kaliyordu, goal_manager
        loglarinda "Bacak basarisiz" (aslinda BASARISIZLIK DEGIL, iptalin
        yan etkisi) SURESIZ tekrarlaniyordu. Cozum: hedef, EN SON
        GONDERILEN hedeften HEDEF_DEGISIM_ESIGI_M'den AZ farkliysa
        TEKRAR GONDERILMEZ - boylece goal_manager/Nav2 sadece GERCEKTEN
        anlamli bir yon degisikligi oldugunda kesintiye ugrar, aksi halde
        mevcut manevrayi TAMAMLAMASINA izin verilir.

        KESIN COZUM (2026-09-01, bkz. HEDEF_ACI_DEGISIM_ESIGI_DERECE notu):
        SADECE XY mesafesi yeterli degil - uzun hedef_mesafe'lerde kucuk
        aci gurultusu esigi kolayca asiyordu. Artik republish icin HEM
        XY mesafesi HEM aci farki esigin ALTINDA kalmali (ikisinden biri
        esigi asarsa GERCEKTEN farkli bir hedef/yon var demektir)."""
        if hedef_yon_yaw_odom is None:
            hedef_yon_yaw_odom = hedef_yaw_odom

        p = self._son_odom.pose.pose.position
        hedef_x = p.x + hedef_mesafe * math.cos(hedef_yaw_odom)
        hedef_y = p.y + hedef_mesafe * math.sin(hedef_yaw_odom)

        if self._son_gonderilen_hedef is not None:
            fark_m = math.hypot(hedef_x - self._son_gonderilen_hedef[0],
                                 hedef_y - self._son_gonderilen_hedef[1])
            fark_aci = abs(math.degrees(math.atan2(
                math.sin(hedef_yaw_odom - self._son_gonderilen_hedef_aci),
                math.cos(hedef_yaw_odom - self._son_gonderilen_hedef_aci))))
            if fark_m < HEDEF_DEGISIM_ESIGI_M and fark_aci < HEDEF_ACI_DEGISIM_ESIGI_DERECE:
                return  # onemli bir degisiklik yok, goal_manager'i gereksiz KESME

        self._son_gonderilen_hedef = (hedef_x, hedef_y)
        self._son_gonderilen_hedef_aci = hedef_yaw_odom
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = hedef_x
        goal.pose.position.y = hedef_y
        goal.pose.orientation.z = math.sin(hedef_yon_yaw_odom / 2.0)
        goal.pose.orientation.w = math.cos(hedef_yon_yaw_odom / 2.0)
        self._goal_pub.publish(goal)

    def _egim_sirasinda_git(self, roll):
        """Egim/rampa gecisinde (pitch esigi asildi) LIDAR'in yan taraflari
        guvenilir gormedigi bilindigi icin (bkz. dosya basi notu) gap-
        following DEVRE DISI - sadece MEVCUT yonde duz gidilir, roll'e gore
        SINIRLI bir duzeltme uygulanir."""
        on_mesafe = self._on_mesafe()
        if on_mesafe is not None and on_mesafe < DUZ_GIDIS_ON_MESAFE_ESIGI_M:
            self.get_logger().warn(
                '⛰️ Eğimde tam önde engel (< %.1fm) - hedef GÖNDERİLMEDİ.' % DUZ_GIDIS_ON_MESAFE_ESIGI_M)
            return

        duzeltme = 0.0
        if abs(roll) > math.radians(ROLL_DUZELTME_ESIGI_DERECE):
            maks = math.radians(ROLL_DUZELTME_MAKS_DERECE)
            duzeltme = max(min(ROLL_DUZELTME_YONU * ROLL_DUZELTME_KAZANC * roll, maks), -maks)
            self.get_logger().warn(
                f'⚠️ Eğimde roll={math.degrees(roll):+.1f}° - yön düzeltmesi '
                f'{math.degrees(duzeltme):+.1f}° uygulanıyor.')

        yaw = _yaw_of(self._son_odom.pose.pose.orientation)
        self._hedef_gonder(yaw + duzeltme, DUZ_GIDIS_MESAFE_M)
        self.get_logger().info(
            f'⛰️ Eğim algılandı - LIDAR yan taraf yerine DÜZ gidiş '
            f'(+{DUZ_GIDIS_MESAFE_M:.1f}m, düzeltme={math.degrees(duzeltme):+.1f}°).')

    def _udonus_isle(self):
        """Duvar/cikmaz TAM ONDE tespit edildi. Kilitli bir yon yoksa
        odometri-tabanli karari VERIR ve KILITLER (bkz. dosya basi notu).
        Kilitli yonun kenarina en yakin gecerli bosluga hedef gonderir.
        True: bu tick islendi (normal gap-following calismasin). False:
        gecerli bir bosluk yok, hedef gonderilmedi (yine de islendi
        sayilir - normal moda DUSMEZ, cunku duvar hala tespit ediliyor)."""
        simdi = time.monotonic()
        if self._udonus_kilitli_yon is None:
            self._udonus_kilitli_yon = self._udonus_yon_karari_ver()
            self._udonus_kilit_zamani = simdi
            self.get_logger().warn(
                f"↩️ Duvar/çıkmaz algılandı - U dönüşü yönü KİLİTLENDİ: "
                f"{'SOL' if self._udonus_kilitli_yon > 0 else 'SAĞ'} "
                f"(odometri yaw eğilimine göre)."
            )

        gecen = simdi - self._udonus_kilit_zamani
        if gecen > UDONUS_ZAMAN_ASIMI_S:
            self.get_logger().error(
                f'🛑 U dönüşü {UDONUS_ZAMAN_ASIMI_S:.0f} saniyedir çözülemedi - '
                'hedef GÖNDERİLMİYOR, müdahale gerekebilir.')
            return

        hedef_aci = math.radians(UDONUS_HEDEF_ACISI_DERECE) * self._udonus_kilitli_yon
        gap = self._en_genis_gap(hedef_aci=hedef_aci)
        if gap is None:
            self.get_logger().warn('↩️ U dönüşü sırasında geçerli boşluk bulunamadı - bekleniyor.')
            return

        on_merkezli_aci, acik_mesafe = gap
        hedef_mesafe = min(UDONUS_HEDEF_MESAFESI_M,
                            max(acik_mesafe - UDONUS_GUVENLIK_PAYI_M, MIN_HEDEF_MESAFESI_M))
        yaw = _yaw_of(self._son_odom.pose.pose.orientation)
        # ORIENTATION, hedef POZISYONA degil ENGELE PARALEL (mevcut yondan
        # kilitli yone dogru 90 derece donuk) - bkz. dosya basi notu
        # (UDONUS_HEDEF_MESAFESI_M ustunde) - "duvara bakan" degil "duvar
        # boyunca donmeye hazir" bir son poz hedeflenir.
        hedef_yon_yaw = yaw + (math.pi / 2.0) * self._udonus_kilitli_yon
        self._hedef_gonder(yaw + on_merkezli_aci, hedef_mesafe, hedef_yon_yaw_odom=hedef_yon_yaw)
        self.get_logger().info(
            f"↩️ U dönüşü ({'SOL' if self._udonus_kilitli_yon > 0 else 'SAĞ'}) devam - "
            f'seçilen yön {math.degrees(on_merkezli_aci):+.0f}°, hedef +{hedef_mesafe:.1f}m, '
            f'yönelim engele paralel ({math.degrees(hedef_yon_yaw):+.0f}°)'
        )

    def _tick(self):
        if not self._aktif:
            return
        if self._son_scan is None or self._son_odom is None:
            return

        pitch = _pitch_of(self._son_odom.pose.pose.orientation)
        if abs(pitch) > math.radians(PITCH_ESIGI_DERECE):
            roll = _roll_of(self._son_odom.pose.pose.orientation)
            self._egim_sirasinda_git(roll)
            return

        # U DONUSU/CIKMAZ KONTROLU (bkz. dosya basi notu) - normal gap-
        # following'den ONCE calisir.
        if self._on_duvar_var_mi():
            self._udonus_isle()
            return

        if self._udonus_kilitli_yon is not None:
            self.get_logger().info('↩️ U dönüşü tamamlandı - normal serbest yön takibine dönülüyor.')
            self._udonus_kilitli_yon = None
            self._udonus_kilit_zamani = None

        gap = self._en_genis_gap()
        if gap is None:
            self.get_logger().warn(
                '⚠️ Serbest yön takibi: yeterince geniş açık yön bulunamadı, '
                'hedef GÖNDERİLMEDİ (bekleniyor).')
            return

        on_merkezli_aci, acik_mesafe = gap
        if acik_mesafe >= ACIK_ONDE_ENGEL_YOK_ESIGI_M:
            hedef_mesafe = UZAK_HEDEF_MESAFESI_M
        else:
            hedef_mesafe = max(acik_mesafe - NORMAL_GUVENLIK_PAYI_M, MIN_HEDEF_MESAFESI_M)

        yaw = _yaw_of(self._son_odom.pose.pose.orientation)
        hedef_yaw_odom = yaw + on_merkezli_aci
        self._hedef_gonder(hedef_yaw_odom, hedef_mesafe)

        self.get_logger().info(
            f'🧭 Açık yön: {math.degrees(on_merkezli_aci):+.0f}° '
            f'(mesafe~{acik_mesafe:.1f}m) -> hedef +{hedef_mesafe:.1f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SerbestYonTakipcisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
