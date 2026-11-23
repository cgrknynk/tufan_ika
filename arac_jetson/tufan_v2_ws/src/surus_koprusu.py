#!/usr/bin/env python3
"""Nav2 <-> arduino_motor_kontrol.py arasi surus koprusu.

/surus_modu (std_msgs/String, "OTONOM"/"MANUEL") degerine gore:
  - OTONOM: /cmd_vel_gated (nav2/MPPI ciktisinin nav2_cmd_vel_gate.py'den
    LiDAR guvenlik kontrolunden gecirilmis hali) dinlenip diferansiyel
    kinematikle sol/sag PWM'e cevrilir ve /palet_hizlari
    (std_msgs/Float32MultiArray, [-255, 255]) olarak yayinlanir.
    NOT (otonom entegrasyonu): oncesinde dogrudan /cmd_vel dinleniyordu;
    goal_manager_node.py + nav2_cmd_vel_gate.py eklenince, Nav2'nin ham
    ciktisi motorlara gitmeden once bagimsiz bir LiDAR kontrolunden gecsin
    diye TEK SATIR degisti (asagida _otonom_cb aboneligi). MANUEL mod,
    PWM donusumu, deadzone ve /surus_modu sozlesmesi HICBIR SEKILDE
    degismedi.
  - MANUEL: bu dugum /palet_hizlari'na HIC YAYIN YAPMAZ. Gercek yer istasyonu
    (tufan_yer_istasyonu) manuel surusu /motor_cmd_vel gibi ara bir Twist
    konusu UZERINDEN DEGIL, dogrudan /palet_hizlari'na kendi PWM degerlerini
    yayinlayarak yapiyor (kendi deadzone/rampa/IMU-egim mantigiyla). Bu dugum
    OTONOM disinda da yayin yaparsa, iki yayinci ayni konuda yariar ve
    manuel surus komutlari araya giren sifir degerlerle kesilir/titrer.

arduino_motor_kontrol.py'a KESINLIKLE dokunulmuyor; bu dugum sadece onun
zaten dinledigi /palet_hizlari formatinda veri uretir.
"""

import time

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, String


class SurusKoprusu(Node):
    def __init__(self):
        super().__init__('surus_koprusu')

        self.declare_parameter('iz_genisligi', 1.15)  # robot.xacro wheel_separation ile ayni
        self.declare_parameter('maks_hiz_ms', 0.4)     # nav2_params.yaml vx_max ile ayni
        self.declare_parameter('komut_timeout', 0.5)
        self.declare_parameter('yayin_frekansi', 20.0)
        # Motor "olu bolge" (deadzone): gercek araçta motorlar bu PWM'in
        # altinda fiziksel olarak donmuyor - dusuk hizlarda arac hic
        # hareket etmiyordu. Sifir olmayan her komut en az bu degere
        # yukseltiliyor (bkz. _deadzone_uygula).
        self.declare_parameter('min_pwm', 90.0)
        # Guvenlik icin ust seyir hizi siniri: [-255,255] olceginde degil, telafi
        # sonrasi gonderilen gercek PWM'in ust siniri (duvara carpma sonrasi
        # dusuruldu - once 255'ti).
        self.declare_parameter('max_pwm', 110.0)
        # DIK EGIMDE TAM GAZ (2026-09-07, kullanici istegi: "dik egimde
        # hizi otonomdayken fule cek, duz yolda normal suanki hizla
        # gitsin"). Yukaridaki max_pwm (110) duz yol seyir tavani -
        # rampaya tirmanmak icin yetersiz kalabiliyor. Govde yukari
        # egildiginde tavan gecici olarak egim_max_pwm'e cikarilir;
        # min_pwm ve diferansiyel kinematik AYNEN korunur, sadece UST
        # SINIR degisir (yani direksiyon/donus yetenegi bozulmaz).
        #
        # SADECE TIRMANISTA: burun ASAGI iken (inis) tavan YUKSELTILMEZ -
        # inise tam gazla girmek tehlikelidir. Esik, projenin diger
        # egim mantiklariyla (egim_costmap_ayarlayici.py,
        # on_bosluk_nokta_atici.py) AYNI: 3 derece.
        #
        # *** IKI SART BIRDEN (2026-09-07, kullanici: "otonomda bazen cok
        # hizli gidiyor; SADECE 8. tabelayi gordugunde VE imu yukari
        # aciya geldiginde hiz artsin") ***
        # Tek basina egim esigi YETMEZ: parkurdaki tumsek/cukur ya da
        # govde sallanmasi 3 dereceyi kisa sureligine asabiliyor ve arac
        # beklenmedik yerde tam gaza kalkiyordu. Artik tam gaz icin
        # /rampa_etabi (8. tabela dogrulanmis) DE gerekli - yani hiz
        # artisi yalnizca gercekten rampa asamasindayken mumkun.
        # rampa_etabi_gerekli=False yapilirsa eski (sadece egim)
        # davranisa donulur.
        self.declare_parameter('egim_max_pwm', 255.0)
        self.declare_parameter('egim_esigi_derece', 3.0)
        # Esikte gidip gelmeyi onlemek icin: egim esigin ALTINA dustukten
        # sonra tam gaz bu kadar saniye daha surer (rampa tepesindeki
        # kisa duzlukte gaz kesilip arac takilmasin).
        self.declare_parameter('egim_histerezis_s', 1.5)
        self.declare_parameter('rampa_etabi_gerekli', True)
        # HIZLANMA PARKURU (2026-09-07, kullanici: "2. parkur 11 ile
        # basliyor, 11'i gorunce SON HIZ gitmesi gerekiyor").
        # Rampa gazindan FARKLI: orada egim sarti da araniyor, burada
        # DUZ YOLDA son hiz istenen davranis - bu yuzden ayri bir bayrak
        # ve egim kontrolunden BAGIMSIZ.
        self.declare_parameter('hizlanma_max_pwm', 255.0)

        # --- TAKILMA (STALL) TESPITI VE GAZ TAKVIYESI (2026-09-09) ---
        # SAHA BILDIRIMI: "arac otonom giderken surekli durup kalkiyor ve
        # suya girdiginde arac ilerlemiyor, suya girmeden duruyor".
        #
        # KOK NEDEN: duz zeminde PWM tavani 110, taban 90 -> motorun
        # TOPLAM 20 birimlik yetkisi var (bkz. _deadzone_uygula:
        # 90 + oran*(110-90)). Kuru zeminde yetiyor; SUDA direnc cok daha
        # yuksek oldugu icin 110 PWM araci hareket ettiremiyor - arac su
        # kenarinda duruyor. Karada da tam stall sinirinda gezindigi icin
        # hareket/durma nobetlesiyor ("durup kalkiyor").
        #
        # NEDEN max_pwm'i BUYUTMUYORUZ: kullanici duz yolda mevcut hizi
        # ISTIYOR ("duz yolda normal suanki hizla gitsin") ve daha once
        # "bazen cok hizli gidiyor" diye sikayet etti. Bu yuzden seyir
        # tavani AYNEN KALIYOR; takviye SADECE "komut var ama arac
        # ilerlemiyor" aninda, kademeli olarak devreye giriyor ve arac
        # hareket eder etmez geri cekiliyor. Yani su/comur/cim/kucuk
        # engel gibi TUM yuksek-direnc durumlarini tek mekanizma cozuyor.
        # 2026-09-12 VARSAYILAN **KAPALI**: yarisma oncesi guvenlik karari.
        # Bu ozellik sahada iki kez beklenmedik hizlanmaya karisti (biri
        # kesin: donus agirlikli komutta gaz acti). Guvenlik sinirlari
        # eklendi ama SAHADA DOGRULANMADI - dogrulanana kadar kapali.
        # Suya/comura takilirsa elle acilabilir:
        #   ros2 param set /surus_koprusu takilma_tespiti_aktif true
        self.declare_parameter('takilma_tespiti_aktif', False)
        # /odom durağanken TAM 0.0 donuyor (canli olculdu), yani gurultu
        # tabani yok - esik yine de guvenli tarafta tutuldu.
        self.declare_parameter('takilma_hiz_esigi_ms', 0.03)
        # Bu kadar sure "komut var + hareket yok" ise takilma sayilir.
        # Kalkis gecikmesini (motor atalet) tetiklemeyecek kadar uzun.
        self.declare_parameter('takilma_suresi_s', 1.2)
        # Takviye tavani ve saniyede kac PWM birimi tirmanacagi.
        self.declare_parameter('takilma_max_pwm', 255.0)
        self.declare_parameter('takilma_artis_pwm_s', 70.0)
        # HISTEREZIS: takilmadan CIKIS esigi, girise gore YUKSEK - aksi
        # halde arac esigin iki yaninda titrer (bu projede histerezisi
        # simetrik yapmak daha once taret kilidini bozmustu).
        self.declare_parameter('takilma_birakma_hizi_ms', 0.10)
        # Hareket basladiktan sonra takviyenin geri cekilme suresi.
        self.declare_parameter('takilma_soguma_s', 1.5)
        # --- 2026-09-11 GUVENLIK SINIRLARI (SAHADA YASANDI) ---
        # Arac "hizlanarak olduğu yerde donuyor" halinde bulundu. Log:
        #   TAKILMA: komut 0.63 m/s ama olculen hiz 0.000 m/s
        # Takviye DONUS agirlikli komutta da devreye giriyordu; tam gaz
        # yerinde donmeye donusuyordu. Takviyenin AMACI su/comur/cim gibi
        # yuksek dirence karsi ILERI itmekti - donus icin degil.
        # (a) Komut ILERI agirlikli degilse takviye YOK.
        self.declare_parameter('takilma_ileri_agirlik_orani', 0.6)
        # (b) Takviye en fazla bu kadar surer; sonra ZORLA kesilir ve
        #     soguma_s kadar tekrar acilmaz. Kacak gaz sonsuza kadar
        #     suremesin - "kurtaramiyorsa zorlamak zarar verir".
        self.declare_parameter('takilma_azami_sure_s', 4.0)
        # INISTE TAKVIYE YOK - GUVENLIK. test_egim_gaz.py bu riski yakaladi:
        # inisde odometri "hareket yok" derse (lazer odometrisi ozelliksiz
        # zeminde/suda yanilabilir) takviye YOKUS ASAGI TAM GAZ verirdi.
        # Zaten inisde ek guce ihtiyac YOK - yercekimi calisiyor; orada
        # takilma varsa sebep engeldir ve tam gaz TEHLIKELIDIR.
        self.declare_parameter('takilma_inis_esigi_derece', -3.0)

        # --- INISTE SABIT MINIMUM HIZ (2026-09-11, kullanici istegi:
        # "imu assagiya baktiginda minimum hiz ile insin", sonra "90 ile
        # gitsin") ---
        # Yokus asagi inerken yercekimi zaten ivmelendiriyor; normal seyir
        # tavani (110) uygulanirsa arac hizlanarak iniyor ve rampanin
        # dibinde savrulma/burun vurma riski doguyor. Inis algilaninca
        # PWM buyuklugu SABIT inis_pwm'e kilitlenir.
        #
        # Deger 90 = min_pwm, yani motorun fiziksel olarak donebildigi EN
        # DUSUK seviye - iniste bundan azi zaten hareket uretmez.
        #
        # DIKKAT - TABAN/TAVAN BIRLIKTE AYARLANMALI: _deadzone_uygula
        # formulu  taban + oran*(tavan - taban)  seklinde. SADECE tavani
        # dusurmek yetmez; taban min_pwm'de kalirsa ve tavan tabanin
        # ALTINA inerse formul TERSINE doner (hizlandikca PWM AZALIR).
        # Bu yuzden iniste taban da tavan da inis_pwm yapilir - buyukluk
        # her komutta tam olarak inis_pwm olur.
        #
        # DIREKSIYON: iki palet de ayni BUYUKLUGU alir, ama ISARET komuttan
        # gelmeye devam eder - keskin donuste bir palet negatif olabilir,
        # yani yerinde donme yetkisi KAYBOLMAZ. Sadece oransal direksiyon
        # (hafif kavis) kaybolur; rampa inisi duz oldugu icin kabul edildi.
        # --- TABELASIZ DIK ENGEL: IMU TEK BASINA GAZ ACAR (2026-09-11) ---
        # Kullanici: "dik engel var, dik engelin tabelasini okumayabilir;
        # onda da IMU'dan dik engeli tirmandigimizi anlayabiliriz, yaklasik
        # 10 derece falan yukari cikiyor; IMU yukari kalktikca hiz artsin".
        #
        # NEDEN AYRI BIR ESIK: rampa gazi (255) TABELA sartina baglanmisti
        # cunku kullanici "bazen cok hizli gidiyor" demisti - kucuk
        # tumsekler/egimler (3-5 derece) tam gazi tetiklemesin diye. O
        # koruma AYNEN DURUYOR. Buraya eklenen, tabeladan BAGIMSIZ bir
        # esik: gercek bir dik engelde gaz tabela okunmasa bile acilir.
        # Esik 5 derece (kullanici: "egim 5 derecenin uzerine cikmadikca
        # hiz artmasin, 5 derece ve uzeri egimlerde hiz artsin").
        #
        # ORANTILI: "kalktikca artsin" istendigi icin bu yol ikili (ac/kapa)
        # degil - esikten itibaren tavan, tam_gaz_derece'ye kadar dogrusal
        # olarak max_pwm'den egim_max_pwm'e tirmanir.
        # 2026-09-12 VARSAYILAN **KAPALI**: ayni guvenlik karari. Tabelasiz
        # dik engelde gaz acmasi ISTENEN davranis ama sahada test
        # edilmedi; tabela yolu (8. tabela + egim) ZATEN CALISIYOR ve
        # rampayi cikariyor. Denemek icin:
        #   ros2 param set /surus_koprusu imu_tek_basina_gaz_aktif true
        self.declare_parameter('imu_tek_basina_gaz_aktif', False)
        self.declare_parameter('imu_tek_basina_esigi_derece', 5.0)
        self.declare_parameter('imu_tam_gaz_derece', 12.0)

        self.declare_parameter('inis_sabit_pwm_aktif', True)
        self.declare_parameter('inis_pwm', 90.0)
        # Bu esikten DAHA DIK asagi egimde sabit hiz devreye girer.
        self.declare_parameter('inis_pwm_esigi_derece', -5.0)
        # SU ETABINDA INIS KILIDI KALKAR (2026-09-09): araç suya INEREK
        # giriyor, yani tam da takviyeye ihtiyac duydugu anda govde asagi
        # egik oluyor ve yukaridaki iniş guvenligi takviyeyi kapatirdi.
        # Suda "yokus asagi kacma" riski YOK - su zaten guclu bir fren.
        self.declare_parameter('takilma_su_etabinda_iniste_aktif', True)

        self._iz_genisligi = self.get_parameter('iz_genisligi').value
        self._maks_hiz = self.get_parameter('maks_hiz_ms').value
        self._timeout = self.get_parameter('komut_timeout').value
        self._min_pwm = self.get_parameter('min_pwm').value
        self._max_pwm = self.get_parameter('max_pwm').value
        self._egim_max_pwm = self.get_parameter('egim_max_pwm').value
        self._egim_esigi = self.get_parameter('egim_esigi_derece').value
        self._egim_histerezis = self.get_parameter('egim_histerezis_s').value
        self._egim_derece = 0.0          # + = burun YUKARI (tirmanis)
        self._son_egim_zamani = None     # esigin ustunde gorulen son an
        self._egim_gazi_acik = False     # log tekrarini onlemek icin
        self._rampa_etabi_gerekli = self.get_parameter('rampa_etabi_gerekli').value
        self._rampa_etabi = False        # 8. tabela dogrulandi mi
        self._hizlanma_max_pwm = self.get_parameter('hizlanma_max_pwm').value
        self._takilma_aktif_ayar = bool(self.get_parameter('takilma_tespiti_aktif').value)
        self._takilma_hiz_esigi = float(self.get_parameter('takilma_hiz_esigi_ms').value)
        self._takilma_suresi = float(self.get_parameter('takilma_suresi_s').value)
        self._takilma_max_pwm = float(self.get_parameter('takilma_max_pwm').value)
        self._takilma_artis = float(self.get_parameter('takilma_artis_pwm_s').value)
        self._takilma_birakma_hizi = float(self.get_parameter('takilma_birakma_hizi_ms').value)
        self._takilma_soguma = float(self.get_parameter('takilma_soguma_s').value)
        self._takilma_ileri_orani = float(
            self.get_parameter('takilma_ileri_agirlik_orani').value)
        self._takilma_azami_sure = float(
            self.get_parameter('takilma_azami_sure_s').value)
        self._takviye_basladi = None      # takviyenin ACILDIGI an
        self._takviye_yasak_bitis = 0.0   # azami sure sonrasi yasak

        # CANLI PARAMETRE DESTEGI (2026-09-11): parametreler acilista bir
        # kez okunup onbellege aliniyordu; 'ros2 param set' "successful"
        # diyor ama dugum ESKI degeri kullanmaya devam ediyordu. Sahada
        # takviyeyi acil kapatmak isterken bu ortaya cikti - komut kabul
        # edildi, hicbir sey degismedi. Artik degisiklikler onbellege
        # yansitiliyor.
        self._param_attr = {
            'max_pwm': '_max_pwm',
            'min_pwm': '_min_pwm',
            'egim_max_pwm': '_egim_max_pwm',
            'hizlanma_max_pwm': '_hizlanma_max_pwm',
            'takilma_tespiti_aktif': '_takilma_aktif_ayar',
            'takilma_hiz_esigi_ms': '_takilma_hiz_esigi',
            'takilma_suresi_s': '_takilma_suresi',
            'takilma_max_pwm': '_takilma_max_pwm',
            'takilma_artis_pwm_s': '_takilma_artis',
            'takilma_birakma_hizi_ms': '_takilma_birakma_hizi',
            'takilma_soguma_s': '_takilma_soguma',
            'takilma_inis_esigi_derece': '_takilma_inis_esigi',
            'takilma_ileri_agirlik_orani': '_takilma_ileri_orani',
            'takilma_azami_sure_s': '_takilma_azami_sure',
            'inis_sabit_pwm_aktif': '_inis_sabit_aktif',
            'inis_pwm': '_inis_pwm',
            'inis_pwm_esigi_derece': '_inis_pwm_esigi',
            'imu_tek_basina_gaz_aktif': '_imu_tek_aktif',
            'imu_tek_basina_esigi_derece': '_imu_tek_esigi',
            'imu_tam_gaz_derece': '_imu_tam_gaz',
        }
        self._takilma_inis_esigi = float(
            self.get_parameter('takilma_inis_esigi_derece').value)
        self._imu_tek_aktif = bool(self.get_parameter('imu_tek_basina_gaz_aktif').value)
        self._imu_tek_esigi = float(self.get_parameter('imu_tek_basina_esigi_derece').value)
        self._imu_tam_gaz = float(self.get_parameter('imu_tam_gaz_derece').value)
        self._imu_tek_logu = False
        self._inis_sabit_aktif = bool(self.get_parameter('inis_sabit_pwm_aktif').value)
        self._inis_pwm = float(self.get_parameter('inis_pwm').value)
        self._inis_pwm_esigi = float(self.get_parameter('inis_pwm_esigi_derece').value)
        self._inis_logu = False
        self._takilma_suda_inis = bool(
            self.get_parameter('takilma_su_etabinda_iniste_aktif').value)
        self._su_etabi = False
        # takilma durumu
        self._olculen_hiz = 0.0          # |lineer| + |acisal| bileseni
        self._takilma_baslangici = None  # komut var + hareket yok baslangici
        self._takviye_pwm = 0.0          # su anki ek PWM (0 = kapali)
        self._takviye_zamani = time.time()
        self._takilma_logu = False
        self._hizlanma_etabi = False     # 11. tabela -> EndOfEleven arasi
        self._hizlanma_gazi_acik = False

        # Guvenli varsayilan: arayuzden mod bilgisi gelene kadar MANUEL kabul et
        # (bilinmeyen durumda otonom surusu asla varsayma).
        self._mod = 'MANUEL'

        self._otonom_twist = Twist()
        self._otonom_zaman = None

        self.create_subscription(String, '/surus_modu', self._mod_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_gated', self._otonom_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Bool, '/rampa_etabi', self._rampa_etabi_cb, 10)
        self.create_subscription(Bool, '/hizlanma_etabi', self._hizlanma_cb, 10)
        self.create_subscription(Bool, '/su_etabi', self._su_etabi_cb, 10)

        self._palet_pub = self.create_publisher(Float32MultiArray, '/palet_hizlari', 10)

        rate = self.get_parameter('yayin_frekansi').value
        self.create_timer(1.0 / rate, self._dongu)

        self.get_logger().info(
            'surus_koprusu aktif: SADECE OTONOM modda /cmd_vel_gated dinlenip /palet_hizlari '
            'yayinlanir (MANUEL modda tamamen sessiz kalinir, o konuyu yer istasyonu yonetir)'
        )


        # CANLI PARAMETRE DESTEGI (2026-09-11): parametreler acilista bir
        # kez okunup onbellege aliniyordu; 'ros2 param set' "successful"
        # diyor ama dugum ESKI degeri kullanmaya devam ediyordu. Sahada
        # takviyeyi ACIL kapatmak isterken ortaya cikti - komut kabul
        # edildi, hicbir sey degismedi. Artik onbellege yansitiliyor.
        self.add_on_set_parameters_callback(self._param_degisti_cb)
    def _param_degisti_cb(self, params):
        """ros2 param set ile gelen degisiklikleri onbellege yansitir."""
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            attr = self._param_attr.get(prm.name)
            if attr is None:
                continue
            setattr(self, attr, prm.value)
            self.get_logger().warn('Parametre guncellendi: %s = %s'
                                   % (prm.name, prm.value))
        return SetParametersResult(successful=True)

    def _mod_cb(self, msg: String):
        yeni_mod = msg.data.strip().upper()
        if yeni_mod in ('OTONOM', 'MANUEL') and yeni_mod != self._mod:
            self.get_logger().info(f'surus modu degisti -> {yeni_mod}')
        if yeni_mod in ('OTONOM', 'MANUEL'):
            self._mod = yeni_mod

    def _otonom_cb(self, msg: Twist):
        self._otonom_twist = msg
        self._otonom_zaman = time.time()

    def _odom_cb(self, msg: Odometry):
        """Govde ileri ekseninin DUNYA-z bileseni -> tirmanis acisi.

        Euler (roll/pitch) cikarmak yerine donme matrisinin 3. satiri
        kullanilir: bu projede pitch ISARET konvansiyonu defalarca sorun
        cikardi, bu yontem konvansiyondan BAGIMSIZ olarak dogrudur
        (ayni yaklasim on_bosluk_nokta_atici.py'de de kullanildi)."""
        q = msg.pose.pose.orientation
        fz = 2.0 * (q.x * q.z - q.w * q.y)   # ileri eksenin dunya-z bileseni
        fz = max(-1.0, min(1.0, fz))
        self._egim_derece = math.degrees(math.asin(fz))
        if self._egim_derece > self._egim_esigi:
            self._son_egim_zamani = time.time()

        # TAKILMA TESPITI icin GERCEK hiz (bkz. _takviyeyi_guncelle).
        # Lineer VE acisal birlikte: yerinde donerken lineer ~0 olur ama
        # arac TAKILMIS DEGILDIR - ikisini toplamak yanlis pozitifi onler.
        t = msg.twist.twist
        self._olculen_hiz = abs(t.linear.x) + abs(t.angular.z) * (
            self._iz_genisligi / 2.0)

    def _rampa_etabi_cb(self, msg: Bool):
        self._rampa_etabi = bool(msg.data)

    def _su_etabi_cb(self, msg: Bool):
        self._su_etabi = bool(msg.data)

    def _hizlanma_cb(self, msg: Bool):
        yeni = bool(msg.data)
        if yeni != self._hizlanma_gazi_acik:
            self._hizlanma_gazi_acik = yeni
            self.get_logger().warn(
                'HIZLANMA PARKURU %s - PWM tavani %.0f'
                % ('ACIK (11. tabela)' if yeni else 'KAPALI (EndOfEleven)',
                   self._hizlanma_max_pwm if yeni else self._max_pwm))
        self._hizlanma_etabi = yeni

    def _takviyeyi_guncelle(self, komut_hizi_ms, lin_x=None, ang_z=None):
        """Komut var ama arac ilerlemiyorsa PWM tavanina kademeli takviye.

        Doner: tavana EKLENECEK PWM miktari (0 = takviye yok).

        MANTIK:
          * komut yok            -> takviye ANINDA sifirlanir (arac durmali)
          * komut var + hareket  -> takviye soguma suresi icinde geri cekilir
          * komut var + hareket yok, takilma_suresi'nden UZUN sure
                                 -> takviye saniyede takilma_artis kadar buyur

        Takviye 'tavan'a EKLENIR, tavani DEGISTIRMEZ: rampa/hizlanma
        tavanlari oldugu gibi kalir, sadece gerektiginde uzerine eklenir.
        """
        su_an = time.time()
        dt = max(0.0, min(su_an - self._takviye_zamani, 0.5))
        self._takviye_zamani = su_an

        if not self._takilma_aktif_ayar:
            return 0.0

        # (a) SADECE ILERI AGIRLIKLI KOMUTTA. Donus bileseni baskinsa
        # takviye verilmez - yerinde donerken tam gaz TEHLIKELI (sahada
        # yasandi, bkz. takilma_ileri_agirlik_orani notu).
        if lin_x is not None and ang_z is not None:
            ileri = abs(float(lin_x))
            donus = abs(float(ang_z)) * (self._iz_genisligi / 2.0)
            toplam = ileri + donus
            if toplam > 1e-6 and (ileri / toplam) < self._takilma_ileri_orani:
                self._takilma_baslangici = None
                self._takviye_pwm = 0.0
                self._takviye_basladi = None
                if self._takilma_logu:
                    self.get_logger().info(
                        'Komut DONUS agirlikli - takilma takviyesi verilmiyor.')
                    self._takilma_logu = False
                return 0.0

        # (b) AZAMI SURE: takviye bu kadar surduyse zorla kesilir ve
        # soguma boyunca tekrar acilmaz.
        if self._takviye_basladi is not None and (
                su_an - self._takviye_basladi) > self._takilma_azami_sure:
            self.get_logger().warn(
                'Takilma takviyesi %.0f sn surdu - KESILDI (kurtaramiyor). '
                '%.0f sn tekrar denenmeyecek.'
                % (self._takilma_azami_sure, self._takilma_soguma))
            self._takviye_pwm = 0.0
            self._takviye_basladi = None
            self._takilma_baslangici = None
            self._takilma_logu = False
            self._takviye_yasak_bitis = su_an + self._takilma_soguma
            return 0.0
        if su_an < self._takviye_yasak_bitis:
            return 0.0

        # INIS GUVENLIGI (bkz. takilma_inis_esigi_derece): asagi egimde
        # takviye ASLA verilmez ve varsa ANINDA kesilir.
        if (self._egim_derece < self._takilma_inis_esigi
                and not (self._su_etabi and self._takilma_suda_inis)):
            self._takilma_baslangici = None
            self._takviye_pwm = 0.0
            if self._takilma_logu:
                self.get_logger().info(
                    'INIS (%.1f derece) - takilma takviyesi kapatildi (guvenlik).'
                    % self._egim_derece)
                self._takilma_logu = False
            return 0.0

        # Komut yoksa takilma diye bir sey YOKTUR - arac zaten durmali.
        # (Stop tabelasi, rampa beklemesi, gorev vetosu vb. hepsi komutu
        # sifirlar; takviyenin oralarda devreye girmesi TEHLIKELI olurdu.)
        if komut_hizi_ms < 1e-3:
            self._takilma_baslangici = None
            self._takviye_pwm = 0.0
            if self._takilma_logu:
                self._takilma_logu = False
            return 0.0

        hareket_ediyor = self._olculen_hiz >= (
            self._takilma_birakma_hizi if self._takviye_pwm > 0.0
            else self._takilma_hiz_esigi)

        if hareket_ediyor:
            self._takilma_baslangici = None
            if self._takviye_pwm > 0.0:
                # Kademeli geri cekilme: aniden kesersek arac tekrar
                # takilir ve "durup kalkma" nobetlesmesi baslar.
                azalma = (self._takilma_max_pwm / max(self._takilma_soguma, 0.1)) * dt
                self._takviye_pwm = max(0.0, self._takviye_pwm - azalma)
                if self._takviye_pwm == 0.0:
                    self._takviye_basladi = None
                if self._takviye_pwm == 0.0 and self._takilma_logu:
                    self.get_logger().info(
                        'Takilma bitti - takviye kapandi, normal seyire donuldu.')
                    self._takilma_logu = False
            return self._takviye_pwm

        # Hareket YOK
        if self._takilma_baslangici is None:
            self._takilma_baslangici = su_an
            return self._takviye_pwm
        if (su_an - self._takilma_baslangici) < self._takilma_suresi:
            return self._takviye_pwm

        if self._takviye_basladi is None:
            self._takviye_basladi = su_an
        self._takviye_pwm = min(self._takviye_pwm + self._takilma_artis * dt,
                                max(0.0, self._takilma_max_pwm - self._max_pwm))
        if not self._takilma_logu:
            self.get_logger().warn(
                'TAKILMA: komut %.2f m/s ama olculen hiz %.3f m/s - '
                'PWM takviyesi devrede (tavan %.0f + %.0f).'
                % (komut_hizi_ms, self._olculen_hiz, self._max_pwm,
                   self._takviye_pwm))
            self._takilma_logu = True
        return self._takviye_pwm

    def _guncel_max_pwm(self):
        """PWM ust siniri: hizlanma parkuru > rampa gazi > normal seyir."""
        # PARKUR 2 (HIZLANMA): 11. tabeladan EndOfEleven'a kadar KOSULSUZ
        # son hiz - egim sarti aranmaz (duz hizlanma pisti).
        if self._hizlanma_etabi:
            return self._hizlanma_max_pwm
        egim_var = (
            self._son_egim_zamani is not None
            and (time.time() - self._son_egim_zamani) < self._egim_histerezis
        )
        tirmaniyor = egim_var and (self._rampa_etabi or not self._rampa_etabi_gerekli)
        if tirmaniyor != self._egim_gazi_acik:
            self._egim_gazi_acik = tirmaniyor
            if tirmaniyor:
                self.get_logger().warn(
                    'RAMPA ETABI + DIK EGIM (%.1f derece) - TAM GAZ: '
                    'PWM tavani %.0f -> %.0f'
                    % (self._egim_derece, self._max_pwm, self._egim_max_pwm))
            else:
                self.get_logger().info(
                    'Egim bitti - normal seyir tavanina donuldu (%.0f)' % self._max_pwm)
        if tirmaniyor:
            return self._egim_max_pwm

        # TABELASIZ DIK ENGEL (bkz. imu_tek_basina_esigi_derece notu):
        # tabela gorulmemis olsa bile IMU yeterince yukari kalkmissa gaz
        # ORANTILI olarak acilir. Tabela YOLU YUKARIDA - oraya girmisse
        # buraya hic gelinmez, yani iki yol birbirini ezmez.
        if self._imu_tek_aktif and self._egim_derece >= self._imu_tek_esigi:
            aralik = max(self._imu_tam_gaz - self._imu_tek_esigi, 0.1)
            oran = (self._egim_derece - self._imu_tek_esigi) / aralik
            oran = max(0.0, min(1.0, oran))
            tavan = self._max_pwm + oran * (self._egim_max_pwm - self._max_pwm)
            if not self._imu_tek_logu:
                self._imu_tek_logu = True
                self.get_logger().warn(
                    '⛰️ DIK ENGEL (TABELASIZ): IMU %.1f derece >= %.1f - '
                    'gaz orantili aciliyor (PWM tavani %.0f).'
                    % (self._egim_derece, self._imu_tek_esigi, tavan))
            return tavan

        if self._imu_tek_logu:
            self._imu_tek_logu = False
            self.get_logger().info(
                'Dik engel bitti - normal seyir tavanina donuldu (%.0f).'
                % self._max_pwm)
        return self._max_pwm

    def _canli_mi(self, zaman):
        return zaman is not None and (time.time() - zaman) < self._timeout

    def _dongu(self):
        if self._mod != 'OTONOM':
            # MANUEL modda bu dugum /palet_hizlari'na hic yazmaz - konuyu
            # tamamen yer istasyonuna birakir (bkz. dosya basi aciklamasi).
            return

        lin_x = self._otonom_twist.linear.x if self._canli_mi(self._otonom_zaman) else 0.0
        ang_z = self._otonom_twist.angular.z if self._canli_mi(self._otonom_zaman) else 0.0

        # Diferansiyel kinematik: m/s -> her tekerlek icin m/s
        # NOT: ileri/geri isareti gercekten calisan MANUEL komutuyla dogrulandi
        # (bu yuzden disaridaki ters cevirme -(...) korunuyor, DOKUNULMADI).
        # Sag/sol donus yonu icin bir onceki degisiklik (ang_z teriminin
        # tekerleklere atanmasini degistirmek) geri alindi - o sirada RViz'de
        # gorulen "ters donus" aslinda rf2o'nun titrek/kayan poz tahmininden
        # kaynaklaniyordu (rf2o kaldirildi), gercek komut yonu zaten dogruydu.
        sol_ms = lin_x + (ang_z * self._iz_genisligi / 2.0)
        sag_ms = lin_x - (ang_z * self._iz_genisligi / 2.0)

        # m/s -> [-255, 255] PWM olcegi
        sol_ham = -(max(min((sol_ms / self._maks_hiz) * 255.0, 255.0), -255.0))
        sag_ham = -(max(min((sag_ms / self._maks_hiz) * 255.0, 255.0), -255.0))

        # Olu bolge telafisi: sifir olmayan komutlari [min_pwm, max_pwm] araligina
        # yeniden olcekle (motor min_pwm altinda donmuyor). Tam sifir ise
        # sifir kalir - arac durmasi gerektiginde gercekten durmali.
        # NOT: ani/guclu kalkis darbesi (kisa sureli kalkis_pwm) kaldirildi -
        # motor zaten min_pwm seviyesinde donuyor, ayrica bir darbeye gerek yok.
        tavan = self._guncel_max_pwm()
        # TAKILMA TAKVIYESI: seyir tavani AYNEN korunur, uzerine sadece
        # "komut var ama arac ilerlemiyor" durumunda ek PWM binder.
        komut_hizi = abs(lin_x) + abs(ang_z) * (self._iz_genisligi / 2.0)
        tavan = min(tavan + self._takviyeyi_guncelle(komut_hizi, lin_x, ang_z),
                    self._takilma_max_pwm)

        # INISTE SABIT MINIMUM HIZ (bkz. inis_pwm notu). Hizlanma
        # parkurunda (PARKUR 2) DEVRE DISI - orasi duz hizlanma pisti,
        # olcum gurultusu yuzunden hafif negatif egim okunursa arac
        # yarisin ortasinda 85'e dusmemeli.
        iniste_mi = (self._inis_sabit_aktif
                     and not self._hizlanma_etabi
                     and self._egim_derece <= self._inis_pwm_esigi)
        if iniste_mi != self._inis_logu:
            self._inis_logu = iniste_mi
            if iniste_mi:
                self.get_logger().warn(
                    '⬇️ INIS (IMU %.1f derece) - SABIT MINIMUM HIZ: PWM %.0f'
                    % (self._egim_derece, self._inis_pwm))
            else:
                self.get_logger().info(
                    'Inis bitti - normal seyir tavanina donuldu (%.0f).'
                    % self._max_pwm)

        if iniste_mi:
            taban = tavan_i = self._inis_pwm
        else:
            taban, tavan_i = self._min_pwm, tavan

        sol_pwm = self._deadzone_uygula(sol_ham, tavan_i, taban)
        sag_pwm = self._deadzone_uygula(sag_ham, tavan_i, taban)

        msg = Float32MultiArray()
        msg.data = [float(sol_pwm), float(sag_pwm)]
        self._palet_pub.publish(msg)

    def _deadzone_uygula(self, ham_pwm, tavan=None, taban=None):
        """Sifir olmayan komutu [taban, tavan] araligina yeniden olcekler.

        taban PARAMETRE oldu (2026-09-11): iniste taban=tavan=inis_pwm
        verilerek buyukluk SABITLENIR. Varsayilan min_pwm - motor bunun
        altinda fiziksel olarak donmuyor."""
        if abs(ham_pwm) < 1e-6:
            return 0.0
        if tavan is None:
            tavan = self._max_pwm
        if taban is None:
            taban = self._min_pwm
        isaret = 1.0 if ham_pwm > 0 else -1.0
        oran = min(abs(ham_pwm) / 255.0, 1.0)
        olcekli = taban + oran * (tavan - taban)
        return isaret * olcekli


def main(args=None):
    rclpy.init(args=args)
    node = SurusKoprusu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
