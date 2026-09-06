import sys
import json
import math
import time
# --- ÇEKİRDEK ROS2 / LİDAR / IMU (rota özelliklerinden BAĞIMSIZ) ---
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Imu, NavSatFix
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
except Exception as e:
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None
    qos_profile_sensor_data = None
    LaserScan = None
    Imu = None
    NavSatFix = None
    OccupancyGrid = None
    Odometry = None
    PoseStamped = None
    String = None
    Buffer = None
    TransformListener = None
    ROS2_IMPORT_ERROR = str(e)
else:
    ROS2_IMPORT_ERROR = None

# --- NAV2 ROTA ÖZELLİKLERİ (ayrı try/except: nav2_msgs kurulu değilse
# SADECE rota özelliği devre dışı kalır, lidar/imu etkilenmez!) ---
try:
    from rclpy.action import ActionClient
    from nav_msgs.msg import Path
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
except Exception as e:
    ActionClient = None
    Path = None
    NavigateToPose = None
    GoalStatus = None
    NAV2_IMPORT_ERROR = str(e)
else:
    NAV2_IMPORT_ERROR = None

# --- RTK DURUMU (2026-09-01, kullanıcı isteği: "uydu haritası kısmında
# gpsin hatası ve fix mi float mı olduğu durumu yazsın") - ayrı try/except:
# mavros_msgs kurulu değilse SADECE bu özellik devre dışı kalır. ---
try:
    from mavros_msgs.msg import GPSRAW
except Exception as e:
    GPSRAW = None
    GPSRAW_IMPORT_ERROR = str(e)
else:
    GPSRAW_IMPORT_ERROR = None

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPoint, QRectF, QPointF, QEvent, QTimer
from PyQt5.QtGui import (QPainter, QColor, QPen, QFont, QPolygon, 
                         QLinearGradient, QBrush, QRadialGradient, QPolygonF,
                         QPixmap)
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QFrame, QTabWidget,
                             QListWidget, QListWidgetItem, QDoubleSpinBox,
                             QGridLayout)
from uydu_harita_widget import UyduHaritaWidget

# ==============================================================================
# 1. ROS 2 ARKA PLAN MOTORU (Orijinal Çalışan QThread + Thread-Safe Nav2)
# ==============================================================================
class Ros2GcsMotoru(QThread):
    lidar_sinyal = pyqtSignal(list)
    imu_sinyal = pyqtSignal(float, float, float)
    baglanti_sinyali = pyqtSignal(str)
    
    global_plan_sinyal = pyqtSignal(list)
    local_plan_sinyal = pyqtSignal(list)
    costmap_sinyal = pyqtSignal(list)
    konum_sinyal = pyqtSignal(float, float)
    gnss_sinyal = pyqtSignal(float, float)  # enlem, boylam (/fix -- Here4 RTK)
    rtk_durum_sinyali = pyqtSignal(int, float)  # fix_type (GPSRAW enum), h_acc (metre)
    gps_katki_sinyali = pyqtSignal(str)  # GPS'in konum tahminine anlik katkisi (insan-okunur)
    rota_durum_sinyali = pyqtSignal(str)  # coklu nokta rotasi ilerleme/durum metni
    panel_durum_sinyali = pyqtSignal(dict)  # ayri process'teki kontrol paneli -> arayuz gostergeleri

    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.calisiyor = True
        self.son_imu_gonderim = 0.0
        self.latest_yaw_rad = 0.0
        # /odom'daki GERCEK (mutlak) arac konumu - rota cizgilerini (plan)
        # arac-goreceli (ileri/sol) cevirmek icin sart, bkz. global/local
        # _plan_callback. tf_buffer YERINE duz bir topic aboneligi (asagida) -
        # tf_buffer C++ seviyesinde cökmeye sebep oldugu icin kalici olarak
        # devre disi (bkz. run() icindeki not); bu basit abonelik o riske hic
        # girmiyor.
        self._odom_x = 0.0
        self._odom_y = 0.0
        self.nav_to_pose_client = None
        self.goal_pose_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        # ÇOKLU NOKTA ROTASI (2026-09-01, kullanıcı isteği: "otonom gitmesi
        # için birkaç nokta vermemiz gerekiyor") - bkz. coklu_hedef_gonder.
        # goal_manager_node.py'nin ('/ugv_goal' -> receding-horizon, uzak/
        # görünmeyen hedefler İÇİN TASARLANMIŞ, bkz. o dosyanın başlığı)
        # /ugv_goal_result (SUCCESS/FAILURE/CANCELLED) sonucunu kullanarak
        # sırayla gönderiliyor - bir bacak bitmeden bir SONRAKİ nokta ASLA
        # gönderilmiyor (goal_manager_node zaten yeni hedefi eskisini iptal
        # ederek kabul ediyor, ama sıraya güvenip erken göndermek yerine
        # sonucu BEKLEMEK daha güvenli/öngörülebilir).
        self.ugv_goal_pub = None
        self._rota_kuyrugu = []      # gönderilmeyi bekleyen [(x,y,q_z,q_w), ...]
        self._rota_toplam = 0
        self._rota_index = 0
        # İZOLE TESTTE BULUNDU: SADECE "_rota_kuyrugu boş mu" kontrolü
        # yetersizdi - SON nokta gönderildiğinde kuyruk zaten boşalıyor,
        # o noktanın SONUCU geldiğinde goal_result_callback bunu "hiç rota
        # yokmuş" sanıp AYIKLAMADAN geçiyordu - "TAMAMLANDI" mesajı hiç
        # tetiklenmiyor, GÖNDER/TEMİZLE butonları sonsuza kadar kilitli
        # kalıyordu. Ayrı bir "rota aktif mi" bayrağı bunu çözer.
        self._rota_aktif = False

    def run(self):
        if rclpy is None:
            print("[HARITA] ROS2 kütüphanesi bulunamadığı için harita motoru başlatılamadı.")
            return

        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node('tufan_gcs_master_node')
        # KENDİ ÖZEL EXECUTOR'IMIZ: rclpy.spin_once(node,...) paylaşılan GLOBAL
        # executor'ı kullanır. main.py'deki LidarThread de aynı global executor'a
        # spin ediyordu -> iki thread aynı wait-set'e aynı anda dokununca
        # "IndexError: wait set index too big" ile çöküyordu. Kendi executor'ımızı
        # kullanarak bu thread'i diğer ROS2 thread'lerinden tamamen izole ediyoruz.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # GEÇİCİ OLARAK KAPALI: tf2_ros TransformListener/Buffer eklendikten sonra
        # uygulama iki kez art arda "terminate called without an active exception"
        # (C++ seviyesinde çökme) ile öldü. Kararlılık önceliğimiz olduğu için
        # costmap izi + canlı X/Y konum özelliğini bu satırı tekrar açana kadar
        # devre dışı bırakıyoruz - lidar/imu/rota bundan etkilenmiyor.
        # if Buffer and TransformListener:
        #     self.tf_buffer = Buffer()
        #     self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        # --- LİDAR VE IMU İÇİN ORİJİNAL ÇALIŞAN QOS AYARLARI ---
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        imu_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- NAV2 ROTALARI İÇİN HIZLI QOS ---
        nav2_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Orijinal Çalışan Abonelikler
        self.lidar_sub = self.node.create_subscription(LaserScan, '/scan', self.scan_callback, lidar_qos)
        # Uydu harita ekrani + Taktik LiDAR GNSS yazisi icin gercek GNSS konumu.
        # DUZELTME (2026-09-01, canli bulundu): "/fix" hicbir zaman yayinlanmiyor
        # (Publisher count: 0, canli dogrulandi - "GNSS: sinyal yok" hep
        # gosteriliyordu, RTK Float calisiyor olsa bile). konum_birlestirici.py
        # (arac tarafi) AYNI sorunu daha once cozmustu - dogrudan mavros'un
        # kendi yayinladigi topic + QoS (BEST_EFFORT, mavros varsayilani)
        # kullaniliyor.
        if NavSatFix is not None:
            self.gnss_sub = self.node.create_subscription(
                NavSatFix, '/mavros/global_position/raw/fix', self.gnss_callback,
                qos_profile_sensor_data)
        # RTK DURUMU (2026-09-01, kullanıcı isteği): "Fix mi Float mı" ve
        # yatay hata (h_acc) bilgisi NavSatFix'te yok - bunlar MAVLink'e
        # özel (GPS_FIX_TYPE enum, h_acc) - GPSRAW mesajında geliyor.
        # konum_birlestirici.py'nin zaten kullandığı AYNI kaynak. NOT: bu
        # topic'in yayıncısı RELIABLE (canlı doğrulandı, /mavros/global_
        # position/raw/fix'ten FARKLI) - telemetri_sistemi.py'deki AYNI
        # (varsayılan depth-10 RELIABLE) abonelikle tutarlı tutuluyor.
        if GPSRAW is not None:
            self.gpsraw_sub = self.node.create_subscription(
                GPSRAW, '/mavros/gpsstatus/gps1/raw', self.gpsraw_callback, 10)
        # DÜZELTME (2026-09-05, kullanıcı: "gpsin konum tahminine olan
        # katkısı da yazılsın arayüzde taktik lidar ekranında") -
        # konum_birlestirici.py (araç) artık GPS düzeltmesinin O ANKİ
        # durumunu (bekliyor mu, ne kadar/hangi hızla düzeltiyor, kalan
        # hata) insan-okunur bir metin olarak /gps_katki_durumu'na
        # yayınlıyor - burada doğrudan alıp Taktik Radar'a iletiyoruz.
        if String is not None:
            self.gps_katki_sub = self.node.create_subscription(
                String, '/gps_katki_durumu', self.gps_katki_callback, 10)
            # PANEL DURUMU (2026-09-06): kontrol paneli artık AYRI BİR
            # PROCESS'te okunuyor (kontrol_paneli_node.py - bkz. main.py
            # _kontrol_paneli_baslat'taki GIL notu). Joystick/acil-stop/
            # silah komutlarını O yayınlıyor; arayüz sadece GÖSTERGELER
            # için (bağlantı durumu, buton olayları, pot/PWM, yer
            # bataryası, log) bu JSON durum akışına abone oluyor -
            # gecikmeye duyarsız olduğu için normal QoS yeterli.
            self.panel_durum_sub = self.node.create_subscription(
                String, '/panel_durumu', self.panel_durum_callback, 10)
            # PANEL KOMUT KANALI (2026-09-06): PANEL TESTİ dialogundaki
            # "Merkezi Sıfırla"/"Kaydet ve Uygula" butonları ile ham veri
            # akışının aç/kapası - panel ayrı process olduğu için metot
            # çağrısı yerine bu topic üzerinden iletilir.
            self.panel_komut_pub = self.node.create_publisher(
                String, '/panel_komut', 10)
        # DÜZELTME: bno055 sürücüsü '/bno055/imu' diye bir topic YAYINLAMIYOR
        # (canlı doğrulandı) - füzyonlu/kalibreli veri gerçekte '/imu/data'da.
        # Eski topic adında hiç publisher olmadığı için IMU hiçbir zaman veri almıyordu.
        self.imu_sub = self.node.create_subscription(Imu, '/imu/data', self.imu_callback, imu_qos)
        # Rota (plan) cizgilerini arac-goreceli cevirmek icin gercek konum -
        # bkz. global_plan_callback/local_plan_callback ve _dunya_to_arac.
        if Odometry is not None:
            self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # costmap_callback artik tf_buffer KULLANMIYOR (bkz. asagidaki not) -
        # bu yuzden tf_buffer kapaliyken bile guvenle acilabilir.
        if OccupancyGrid is not None:
            costmap_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.costmap_sub = self.node.create_subscription(
                OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, costmap_qos)

        # /goal_pose yayıncısı: RViz'in "2D Nav Goal" aracıyla aynı standart giriş
        # noktası - bt_navigator bunu doğrudan dinliyor (canlı doğrulandı).
        if PoseStamped is not None:
            self.goal_pose_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
            # /ugv_goal: goal_manager_node.py'nin receding-horizon girişi -
            # ÇOKLU NOKTA rotası bunu kullanır (bkz. coklu_hedef_gonder).
            # /goal_pose'tan FARKLI: uzak/görünmeyen hedefler için tasarlandı
            # (costmap penceresi dışına da gidebilir, kendi içinde ara
            # bacaklara böler) ve /ugv_goal_result (SUCCESS/FAILURE/
            # CANCELLED) ile TAMAMLANMA bildirir - sıradaki noktayı ne zaman
            # göndereceğimizi bununla biliyoruz.
            self.ugv_goal_pub = self.node.create_publisher(PoseStamped, '/ugv_goal', 10)

        # Nav2 Abonelikleri (nav2_msgs kurulu değilse bu kısım atlanır, Lidar/IMU etkilenmez)
        if ActionClient and NavigateToPose and Path:
            self.nav_to_pose_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
            self.global_plan_sub = self.node.create_subscription(Path, '/plan', self.global_plan_callback, nav2_qos)
            self.local_plan_sub = self.node.create_subscription(Path, '/local_plan', self.local_plan_callback, nav2_qos)
            if String is not None:
                # Kullanici istegi (2026-08-31): rota tamamlaninca (varsa
                # basarisiz da olsa) mavi rota cizgisi ekranda SONSUZA KADAR
                # kalmamali - Nav2 hedefe ulasinca /plan'a yeni mesaj
                # YAYINLAMAYI KESIYOR (bos Path de gelmiyor), yani eski
                # cizgi TEMIZLENMEDEN kaliyordu. goal_manager_node.py'nin
                # zaten yayinladigi /ugv_goal_result (SUCCESS/FAILURE)
                # sonucunu dinleyip rota bitince cizgiyi temizliyoruz.
                self.goal_result_sub = self.node.create_subscription(
                    String, '/ugv_goal_result', self.goal_result_callback, 10
                )
            self.baglanti_sinyali.emit("🟢 ROS 2 Aktif: Lidar & IMU & Nav2 Rota")
        else:
            print(f"[HARITA] Nav2 mesaj paketleri (nav2_msgs) bulunamadı, rota özelliği devre dışı: {NAV2_IMPORT_ERROR}")
            self.baglanti_sinyali.emit("🟡 ROS 2 Aktif: Lidar & IMU (Nav2 rota paketleri eksik!)")

        print("🚀 ROS 2 Motoru Başlatıldı: QThread Modu.")

        # ORİJİNAL ÇALIŞAN SPIN DÖNGÜSÜ (Kilitlenmez) - kendi izole executor'ımızla
        while rclpy.ok() and self.calisiyor:
            self.executor.spin_once(timeout_sec=0.005)

    def scan_callback(self, msg):
        # DÜZELTME (2026-09-05, kullanıcı: "lidar ve imu çalıştığını
        # gösterdiğinde çok kasıyor ve arayüz çöktü" - GUI takılma
        # izleyicisiyle KANITLANDI: paintEvent'te LiDAR noktalarını TEK
        # TEK antialiaslı drawEllipse ile çizen döngü 1.7-2.7 SANİYE
        # sürüyordu). TAM bir 360° taramada (~360-720 nokta, HİÇ örnekleme
        # yapılmadan) her nokta ayrı bir antialiaslı QPainter çağrısı -
        # Jetson'ın yazılımsal (donanım hızlandırmasız) çiziminde bu KAT
        # KAT ağır. Bu SADECE GÖRSEL bir gösterge (nav2'nin kendi costmap'i
        # engelden kaçınmayı AYRICA/bağımsız yapıyor, bkz. costmap_callback)
        # - costmap_callback'teki AYNI "adim" (örnekleme) deseniyle veri
        # kaynağında seyreltiliyor, hem sinyal boyutu hem çizim maliyeti
        # ~3 kat azalıyor, gözle fark edilmez (RViz'in kendisi de GPU'da
        # noktaları küçük çiziyor, insan gözü için yoğunluk farkı önemsiz).
        # DÜZELTME 2 (2026-09-05, kullanıcı: "uygulama dondu, force quit
        # ettim" - canlı yakalandı: donma anında Taktik Lidar ekranı
        # açıkken "803 nokta" gösteriyordu). Fark edildi: neredeyse HER
        # ışın geçerli bir mesafe döndürür (açık alanda bile menzil
        # sınırına kadar bir değer gelir, "engel yoğunluğu" ile orantılı
        # DEĞİL) - yani bu ~800 civarı SÜREKLİ/NORMAL çalışma noktası,
        # nadir bir "yoğun ortam" istisnası değildi. adim=3 sabit oranı
        # LiDAR'ın kendi çözünürlüğüne göre hâlâ yüzlerce nokta
        # bırakıyordu. Artık HEM örnekleme sıklaştırıldı HEM DE kesin bir
        # üst sınır var - ortamdan/donanımdan bağımsız, tek kare çizim
        # maliyeti asla belli bir tavanın üzerine çıkamaz.
        adim = 5
        MAKS_NOKTA = 300
        gecerli_noktalar = []
        for i in range(0, len(msg.ranges), adim):
            mesafe = msg.ranges[i]
            if math.isinf(mesafe) or math.isnan(mesafe):
                continue
            if 0.05 < mesafe < 15.0:
                aci = msg.angle_min + i * msg.angle_increment + math.pi
                x = mesafe * math.cos(aci)
                y = mesafe * math.sin(aci)
                gecerli_noktalar.append((x, y))

        if len(gecerli_noktalar) > MAKS_NOKTA:
            ek_adim = len(gecerli_noktalar) // MAKS_NOKTA + 1
            gecerli_noktalar = gecerli_noktalar[::ek_adim]

        # GEÇİCİ PROFİLLEME (2026-09-06) - emit ANI kaydediliyor; GUI
        # tarafındaki slot (HaritaYoneticisi.lidar_guncelle) bunu okuyup
        # "sinyalin kuyrukta ne kadar beklediğini" ölçüyor. Bu değer
        # ZAMANLA ARTIYORSA, Qt sinyal kuyruğu birikiyor demektir (ROS2
        # tarafı GUI'nin işleyebileceğinden hızlı emit ediyor) - bu tam
        # olarak "başta iyi, sonra kötüleşiyor" davranışını üretir.
        self._son_lidar_emit_t = time.perf_counter()
        self.lidar_sinyal.emit(gecerli_noktalar)

    def imu_callback(self, msg):
        q = msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return

        su_an = time.time()
        # DÜZELTME (2026-09-05, kullanıcı: "lidar ve imu çalıştığını
        # gösterdiğinde çok kasıyor ve arayüz çöktü") - bu sinyal Suni
        # Ufuk'un VE Taktik Radar'ın TAM yeniden çizimini (grid, waypoint,
        # kamera bindirmesi) tetikliyor - eski 0.016sn (~62Hz) eşiği,
        # kamera akışı için uyguladığımız ~15Hz render bütçesinin KAT KAT
        # üzerindeydi ve o throttle'ın hiç kapsamadığı TAMAMEN AYRI bir
        # yoldu. IMU 85Hz yayınlayınca bu, saniyede 62 kez tam ekran
        # yeniden çizim demekti - GUI thread'i boğup çökmeye kadar
        # götürüyordu. İnsan gözü için 20Hz zaten akıcı, gereksiz yere
        # 3 katı işlem yapılmasın.
        if su_an - self.son_imu_gonderim < 0.05:
            return
        self.son_imu_gonderim = su_an

        # --- IMU MONTAJ DÜZELTMESİ (fiziksel eksen <-> araç gövde ekseni) ---
        # BNO055 kartı aracın gerçek önüne değil, ~90° yanlış yöne monte
        # edilmiş (kullanıcının ifadesiyle: kartın Y ekseni aracın ÖNÜNE,
        # X ekseni SOLUNA bakıyor). 2026-08-30'da araç tarafında (bkz.
        # konum_birlestirici.py _mount_fix) gerçek fiziksel PITCH testiyle
        # (RViz'de doğrulandı: düzeltme kaldırılınca kaldırma hareketi
        # pitch'e değil ROLL'e yansıyordu) kanıtlanan AYNI düzeltme burada
        # da uygulanıyor: HAM /imu/data kuaterniyonuna, araç gövde
        # çerçevesinde (ART-ÇARPIM/post-multiply) +90° Z-ekseni dönüşü:
        # q_govde = q_imu ⊗ q_montaj  (q_montaj = Z ekseninde +90° dönüş).
        KOK2_2 = 0.7071067811865476  # cos(45°) = sin(45°)
        w1, x1, y1, z1 = q.w, q.x, q.y, q.z
        w = KOK2_2 * (w1 - z1)
        x = KOK2_2 * (x1 + y1)
        y = KOK2_2 * (y1 - x1)
        z = KOK2_2 * (w1 + z1)

        sin_x = 2.0 * (w * x + y * z)
        cos_x = 1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(sin_x, cos_x))

        # NOT: canli testte dogrulandi -> burun yukarida iken bu hesap negatif,
        # burun asagidayken pozitif cikiyordu (beklenenin tersi). Isareti
        # burada ceviriyoruz ki pitch>0 = burun yukari, pitch<0 = burun asagi olsun.
        sin_y = 2.0 * (w * y - z * x)
        if abs(sin_y) >= 1:
            pitch = -math.degrees(math.copysign(math.pi / 2, sin_y))
        else:
            pitch = -math.degrees(math.asin(sin_y))

        sin_z = 2.0 * (w * z + x * y)
        cos_z = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(sin_z, cos_z)
        yaw = math.degrees(yaw_rad)
        # costmap_callback ve TaktikRadarEkrani (hedef tıklama, araç ikonu
        # dönüşü) bunu kullanır - artık /odom ile AYNI (montaj düzeltmeli)
        # referans, bkz. costmap_callback'teki 2026-08-30 notu.
        self.latest_yaw_rad = yaw_rad

        self.imu_sinyal.emit(roll, pitch, yaw)

    def odom_callback(self, msg):
        # GUNCELLEME: eski tf_buffer.lookup_transform('odom','base_footprint')
        # yontemi C++ seviyesinde cökmeye sebep oldugu icin kalici olarak
        # devre disiydi (self.tf_buffer hep None) - bu yuzden arac konumu
        # HICBIR ZAMAN guncellenmiyordu ve rota (plan) cizgileri asagida
        # aciklanan sebeple "boslukta" kaliyordu. Duz /odom aboneligi ayni
        # bilgiyi TF'siz, cökme riski olmadan verir.
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self.konum_sinyal.emit(self._odom_x, self._odom_y)

    def _dunya_to_arac(self, world_x, world_y):
        # costmap_callback'teki (bkz. asagisi) DUNYA -> ARAC GOVDESI donusumun
        # birebir ayni formulu - /plan ve /local_plan HAM /odom (mutlak)
        # koordinatlarinda gelir, ama TaktikRadarEkrani araci HER ZAMAN ekran
        # MERKEZINDE sabit cizer (bkz. paintEvent). Bu donusum olmadan, arac
        # odom sifirindan uzaklastikca rota cizgisi sabit-merkezdeki araç
        # ikonuna gore "boslukta" kalir - canli bildirilen hata buydu.
        dx = world_x - self._odom_x
        dy = world_y - self._odom_y
        yaw = self.latest_yaw_rad
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        ileri = dx * cos_y + dy * sin_y
        sol = -dx * sin_y + dy * cos_y
        return ileri, sol

    def _gecmis_mi(self, world_x, world_y):
        # Bir dunya noktasinin aracin O ANKI yonune gore ARKASINDA kalip
        # kalmadigini soyler (ileri projeksiyon < -0.3m tamponu) - rota
        # cizgisini GERIDEN eksiltmek icin kullanilir (bkz. *_plan_callback).
        # Artik ekrana CIZIM icin degil (o artik SABIT CERCEVE/dunya
        # koordinati kullaniyor, bkz. TaktikRadarEkrani), SADECE bu filtre
        # icin yon hesabi gerekiyor.
        dx = world_x - self._odom_x
        dy = world_y - self._odom_y
        yaw = self.latest_yaw_rad
        ileri = dx * math.cos(yaw) + dy * math.sin(yaw)
        return ileri < -0.3

    def global_plan_callback(self, msg):
        # Kullanici istegi (2026-09-01): arac ilerledikce mavi rota GERIDEN
        # eksilmeli, sadece hedefe varinca tumden silinmemeli. Nav2 zaten
        # 10Hz'de (expected_planner_frequency, nav2_params.yaml) yeniden
        # planlama yapiyor - her yeni /plan mesaji genelde aracin O ANKI
        # konumundan baslar, ama garanti degil (bazi planlayicilar/durumlar
        # eski bir rotayi oldugu gibi tekrar yayinlayabilir). Bu yuzden
        # GECMIS noktalari burada ACIKCA eliyoruz - 10Hz replan ile birlikte
        # bu, "canli eksilen rota" gorunumunu garanti eder.
        # GUNCELLEME (2026-09-01, SABIT CERCEVE): rota cizgisi artik arac-
        # goreceli (ileri,sol) DEGIL, HAM DUNYA (/odom) koordinati olarak
        # yayinlaniyor - TaktikRadarEkrani artik RViz'deki gibi SABIT
        # yonelimli (dunya donmez) bir goruntu ciziyor, sadece kamera araci
        # takip ediyor (bkz. TaktikRadarEkrani.paintEvent). Eskiden aracin
        # KENDISI ekranda hep sabit/donuk gorunuyor, rota da onun etrafinda
        # "boslukta" kalmis gibi duruyordu (canli bildirildi) - artik rota
        # SABIT durur, arac ikonu onun UZERINDE gercekten ilerler/doner.
        poses = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        poses = [(x, y) for x, y in poses if not self._gecmis_mi(x, y)]
        self.global_plan_sinyal.emit(poses)

    def local_plan_callback(self, msg):
        poses = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        poses = [(x, y) for x, y in poses if not self._gecmis_mi(x, y)]
        self.local_plan_sinyal.emit(poses)

    def goal_result_callback(self, msg):
        # SUCCESS veya FAILURE farketmez - o rota denemesi bitti, ekrandaki
        # mavi cizgi artik gecerli/guncel degil, temizlenir (bos liste
        # gonderilince TaktikRadarEkrani.paintEvent'teki "if self.global_plan:"
        # kontrolu zaten cizmeyi kendiliginden birakiyor).
        self.global_plan_sinyal.emit([])
        self.local_plan_sinyal.emit([])

        # ÇOKLU NOKTA ROTASI (bkz. coklu_hedef_gonder): bir bacak bitince
        # (bu callback) SIRADAKİ noktayı gönder - SADECE SUCCESS'te devam
        # edilir, FAILURE/CANCELLED'da rota GÜVENLİ TARAFTA durdurulur
        # (kalan noktalar körlemesine gönderilmez, kullanıcı bilgilendirilir).
        # NOT: "_rota_kuyrugu boş mu" YERİNE "_rota_aktif" kontrol ediliyor -
        # izole testte bulundu: SON nokta gönderildiğinde kuyruk zaten
        # boşalıyor, o noktanın sonucu geldiğinde eski kod bunu "hiç rota
        # yokmuş" sanıp atlıyordu (TAMAMLANDI mesajı hiç tetiklenmiyordu).
        if self._rota_aktif:
            sonuc = (msg.data or "").strip().upper()
            if sonuc == "SUCCESS":
                self._sonraki_hedefi_gonder()
            else:
                kalan = len(self._rota_kuyrugu)
                self._rota_kuyrugu = []
                self._rota_aktif = False
                self.rota_durum_sinyali.emit(
                    f"🛑 Rota DURDURULDU: nokta {self._rota_index}/{self._rota_toplam} "
                    f"{sonuc} oldu - kalan {kalan} nokta GÖNDERİLMEDİ.")

    def coklu_hedef_gonder(self, nokta_listesi):
        """nokta_listesi: [(x, y, q_z, q_w), ...] - SIRAYLA, bir öncekinin
        SUCCESS sonucu gelmeden bir SONRAKİ ASLA gönderilmez (bkz.
        goal_result_callback). Kullanıcı isteği (2026-09-01): "otonom
        gitmesi için birkaç nokta vermemiz gerekiyor"."""
        if not self.ugv_goal_pub:
            self.rota_durum_sinyali.emit("HATA: /ugv_goal yayıncısı başlatılamamış!")
            return
        if not nokta_listesi:
            return
        self._rota_kuyrugu = list(nokta_listesi)
        self._rota_toplam = len(nokta_listesi)
        self._rota_index = 0
        self._rota_aktif = True
        self._sonraki_hedefi_gonder()

    def _sonraki_hedefi_gonder(self):
        if not self._rota_kuyrugu:
            self._rota_aktif = False
            self.rota_durum_sinyali.emit(
                f"✅ Rota TAMAMLANDI - {self._rota_toplam}/{self._rota_toplam} nokta.")
            return
        x, y, q_z, q_w = self._rota_kuyrugu.pop(0)
        self._rota_index += 1

        msg = PoseStamped()
        msg.header.frame_id = 'odom'  # bkz. hedefe_git() - bu araçta TEK gerçek frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = float(q_z)
        msg.pose.orientation.w = float(q_w)
        self.ugv_goal_pub.publish(msg)

        self.rota_durum_sinyali.emit(
            f"🚩 Nokta {self._rota_index}/{self._rota_toplam} gönderildi: "
            f"X={x:.2f} Y={y:.2f}")

    def gnss_callback(self, msg):
        # status.status < 0 (STATUS_NO_FIX) ise konum gecersiz, gonderme.
        if msg.status.status < 0:
            return
        self.gnss_sinyal.emit(msg.latitude, msg.longitude)

    def gpsraw_callback(self, msg):
        # h_acc MAVLink'te milimetre - 0 ise bu donanimda doldurulmuyor
        # demektir (konum_birlestirici.py'deki ayni supheyle canli
        # dogrulandi: bu FC/GNSS alicisinda GERCEKTEN doluyor, örn. 128mm).
        h_acc_m = (msg.h_acc / 1000.0) if msg.h_acc > 0 else -1.0
        self.rtk_durum_sinyali.emit(msg.fix_type, h_acc_m)

    def gps_katki_callback(self, msg):
        self.gps_katki_sinyali.emit(msg.data)

    def panel_komut_gonder(self, komut):
        """Arayüzden ayrı process'teki kontrol paneli node'una komut yollar
        (bkz. kontrol_paneli_node.py::_komut_cb)."""
        pub = getattr(self, 'panel_komut_pub', None)
        if pub is None or String is None:
            return False
        try:
            pub.publish(String(data=str(komut)))
            return True
        except Exception:
            return False

    def panel_durum_callback(self, msg):
        # Ayrı process'teki kontrol paneli node'undan gelen JSON durum
        # (bkz. kontrol_paneli_node.py::durum_yayinla). Bozuk/eksik veri
        # arayüzü ASLA düşürmemeli - sessizce yok sayılır.
        try:
            veri = json.loads(msg.data)
        except Exception:
            return
        if isinstance(veri, dict):
            self.panel_durum_sinyali.emit(veri)

    def costmap_callback(self, msg):
        # local_costmap 'rolling_window: true' ile geliyor (nav2_params.yaml'da
        # dogrulandi) -> arac HER ZAMAN gridin tam merkezinde. Bu sayede TF
        # (odom->base_footprint) lookup'ina hic gerek kalmadan robot konumunu
        # gridin geometrik merkezinden hesaplayabiliyoruz -- eski kod burada
        # tf_buffer kullaniyordu ama tf_buffer/TransformListener kararsizliga
        # (art arda cökme) sebep oldugu icin kalici olarak devre disi (bkz.
        # run() icindeki not); bu yontem o riske hic girmiyor.
        # GUNCELLEME (2026-08-30): Yon icin artik self.latest_yaw_rad (BNO055
        # MONTAJ DUZELTMESI UYGULANMIS yaw - bkz. imu_callback) kullaniliyor.
        # ESKIDEN burada HAM (duzeltmesiz) yaw kullaniliyordu, cunku o zaman
        # konum_birlestirici.py da /odom'u HAM yonelimle yayinliyordu - ikisi
        # tutarliydi. Ama BNO055'in aracin gercek onune degil ~90 derece yanlis
        # yone monte edildigi bulunup konum_birlestirici.py'de DUZELTILDIGINDEN
        # (canli lift testiyle dogrulandi, ayni +90 derece Z-ekseni duzeltmesi),
        # /odom ARTIK duzeltilmis yaw kullaniyor - costmap da aynisini kullanmali,
        # yoksa hedef (goal) yerlesimi ve arac ikonu ile costmap/rota birbirine
        # gore ~90 derece kaymis gorunur (canli bildirildi: "konum sola gidiyor").
        res = msg.info.resolution
        genislik0 = msg.info.width
        yukseklik0 = msg.info.height
        if res <= 0.0 or genislik0 == 0 or yukseklik0 == 0:
            return

        # GUNCELLEME (2026-09-01, SABIT CERCEVE): artik arac govdesine
        # (ileri/sol) CEVRILMIYOR - HAM DUNYA (/odom) koordinati olarak
        # yayinlaniyor (bkz. global_plan_callback'teki ayni tarihli not).
        # Yon (yaw) donusumune artik hic gerek yok - her hucrenin dunya
        # konumu SADECE origin+index*res ile belirleniyor.
        genislik = genislik0
        yukseklik = yukseklik0
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        esik = 60          # 0-100 doluluk; üzeri "engel" sayılır (lethal'e yakın)
        adim = 2            # performans için her 2 hücrede bir örnekle
        veri = msg.data

        noktalar = []
        for j in range(0, yukseklik, adim):
            satir_baslangic = j * genislik
            world_y = origin_y + (j + 0.5) * res
            for i in range(0, genislik, adim):
                if veri[satir_baslangic + i] >= esik:
                    world_x = origin_x + (i + 0.5) * res
                    noktalar.append((world_x, world_y))

        # DÜZELTME (2026-09-05, LiDAR'daki AYNI sabit üst sınır deseni -
        # bkz. scan_callback'teki not) - costmap boyutu/çözünürlüğü ileride
        # değişirse bile tek kare çizim maliyeti asla belli bir tavanın
        # üzerine çıkmasın diye.
        MAKS_NOKTA = 300
        if len(noktalar) > MAKS_NOKTA:
            ek_adim = len(noktalar) // MAKS_NOKTA + 1
            noktalar = noktalar[::ek_adim]

        self.costmap_sinyal.emit(noktalar)

    def hedefe_git(self, x, y, q_z, q_w):
        # RViz'in "2D Nav Goal" aracıyla AYNI yol: /goal_pose'a yayınla.
        # bt_navigator bunu doğrudan dinliyor (canlı doğrulandı) ve kendi içinde
        # navigate_to_pose'u tetikliyor - hem RViz'de goal/plan görünür olur,
        # hem de action client'ı elle yönetmeye gerek kalmaz.
        if not self.goal_pose_pub:
            self.baglanti_sinyali.emit("HATA: /goal_pose yayıncısı başlatılamamış!")
            return

        # DÜZELTME: Bu robotta 'map' frame'i hiç yok (AMCL/map_server çalışmıyor,
        # Nav2 sadece 'odom' frame'inde çalışıyor) - canlı test ettim, frame_id='map'
        # ile hedef Nav2 tarafından SESSİZCE REDDEDİLİYORDU ("Goal was rejected"),
        # bu yüzden rota hiçbir zaman planlanmıyor, RViz'de de hiç görünmüyordu.
        # GÜNCELLEME (2026-09-01, SABİT ÇERÇEVE): eskiden 'base_footprint'
        # (araç-göreceli) kullanılıyordu çünkü (x,y) TaktikRadarEkrani'nde
        # aracın O ANKİ konumuna göre hesaplanıyordu - ama bu, hedefin
        # "istenen NOKTAYA değil, tıklama anında aracın neresindeyse ORAYA
        # göre bir OFSETE" gitmesi riskini taşıyordu (canlı bildirilen
        # "goal_pose istediğim noktaya gitmiyor" şikayetiyle tutarlı).
        # Nav2'nin TÜM stack'i zaten global_frame=odom kullanıyor
        # (nav2_params.yaml'da doğrulandı) - artık (x,y) TaktikRadarEkrani'nde
        # DOĞRUDAN mutlak /odom koordinatı olarak hesaplanıyor, burada da
        # frame_id='odom' ile HİÇBİR dönüşüme uğramadan yayınlanıyor -
        # tıklanan nokta = hedefin gideceği nokta, garantili.
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(q_z)
        msg.pose.orientation.w = float(q_w)

        self.goal_pose_pub.publish(msg)
        self.baglanti_sinyali.emit(f"✅ Hedef /goal_pose'a Gönderildi: X={x:.2f}, Y={y:.2f}")

    def stop(self):
        self.calisiyor = False
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        self.quit()
        self.wait()


# ==============================================================================
# 2. SOL PANEL: GERÇEK ZAMANLI SUNİ UFUK GÖSTERGESİ
# ==============================================================================
class SuniUfukEkrani(QFrame):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(300, 300)
        self.setStyleSheet("background-color: #11141a; border: 2px solid #222b38; border-radius: 15px;")
        self.roll = 0.0
        self.pitch = 0.0
        # SİLAH KAMERASI ARKA PLANI (2026-09-02, kullanıcı isteği düzeltmesi:
        # "ufuk kısmının parametreleri aynen duracak fakat o kahverengi ve
        # mavi olan kısma görüntü gelecek, görüntünün üstünde yine ufuk
        # yazıları olacak") - ilk denemede AYRI bir kutuya konmuştu, bu
        # YANLIŞTI - istenen, gökyüzü(mavi)/yer(kahverengi) dolgusunun
        # YERİNİ gerçek kamera görüntüsünün alması, pitch ladder/derece
        # yazıları/uçak sembolü/çerçeve AYNEN kalması (bkz. paintEvent).
        self.silah_pixmap = None

    def guncelle_veri(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        # DÜZELTME (2026-09-05, "IMU çalışınca çok kasıyor") - eskiden
        # görünürlükten BAĞIMSIZ her çağrıda self.update() çağrılıyordu -
        # TaktikRadarEkrani.guncelle_veri'deki AYNI korumaya (bkz. orada)
        # kavuşturuldu, görünmeyen sayfada boşuna çizim yapılmasın.
        if self.isVisible():
            self.update()

    def silah_kare_guncelle(self, pixmap):
        """main.py::video_ekrana_bas'tan çağrılır (SADECE Navigasyon
        sayfasındayken - bkz. oradaki performans notu) - gökyüzü/yer
        dolgusunun yerini alacak kareyi günceller."""
        self.silah_pixmap = pixmap
        if self.isVisible():
            self.update()

    def paintEvent(self, event):
        # DÜZELTME (2026-09-05, segfault kökeni bulundu): QPainter'ı try/finally
        # ile sarmalıyoruz. uydu_harita_widget.py'de AYNI kalıp (paintEvent
        # içinde erken return / istisna olursa ressam.end() hiç çağrılmıyordu)
        # gerçek bir SIGSEGV'e yol açtı (log'da tekrar tekrar "QBackingStore::
        # endPaint() called with active painter" uyarısı, ardından çökme).
        # Burada henüz erken-return yok ama gövde uzun ve harici veri (IMU
        # açıları, pixmap boyutu) üzerinde işlem yapıyor - önlem olarak
        # aynı güvenli kalıba alınıyor.
        ressam = QPainter(self)
        try:
            ressam.setRenderHint(QPainter.Antialiasing)
            self._paintEvent_govde(ressam)
        finally:
            ressam.end()

    def _paintEvent_govde(self, ressam):
        g = self.width()
        y = self.height()
        merkez_x, merkez_y = g / 2, y / 2
        yaricap = min(g, y) / 2 - 20

        ressam.save()
        ressam.translate(merkez_x, merkez_y)
        ressam.setClipRect(int(-yaricap), int(-yaricap), int(yaricap * 2), int(yaricap * 2))
        
        ressam.rotate(self.roll)
        pitch_piksel = self.pitch * 2.5

        if self.silah_pixmap is not None and not self.silah_pixmap.isNull():
            # DÜZELTME (2026-09-02): gökyüzü(mavi)/yer(kahverengi) dolgusu
            # YERİNE silah kamerası görüntüsü - AYNI translate/rotate
            # bloğunun İÇİNDE olduğu için roll ile döner, pitch ile AYNI
            # şekilde kayar (eski dolgu neresi kaplıyorsa görüntü de orayı
            # kaplar).
            # DÜZELTME v2 (canlı bildirildi: "kare olduğu için çok yakını
            # gösteriyor, zoom yapmış gibi") - v1 görüntüyü ORTADAN KARE
            # KIRPIYORDU (800x800 hedefe sığdırmak için) - geniş (16:9/4:3)
            # bir kamera görüntüsünü kareye kırpmak GENİŞLİĞİN BÜYÜK
            # KISMINI (16:9'da ~%44'ünü) atıyordu, bu da "yakınlaştırılmış"
            # görünüme yol açıyordu. Artık HİÇ KIRPMA YOK - kaynağın
            # TAMAMI çiziliyor, hedef dikdörtgen kaynağın EN-BOY ORANINI
            # KORUYARAK büyütülüyor (kısa kenarı yine 800px - eski kare
            # dolgunun "her roll açısında görünür alanı kaplar" garantisini
            # koruyor, sadece uzun kenar orana göre büyüyor, KIRPMA yok).
            kaynak_boyut = self.silah_pixmap.size()
            oran = kaynak_boyut.width() / max(1, kaynak_boyut.height())
            kisa_kenar = 800.0
            if oran >= 1.0:
                hedef_g, hedef_y = kisa_kenar * oran, kisa_kenar
            else:
                hedef_g, hedef_y = kisa_kenar, kisa_kenar / oran
            hedef = QRectF(-hedef_g / 2.0, -hedef_y / 2.0 + pitch_piksel, hedef_g, hedef_y)
            ressam.drawPixmap(hedef, self.silah_pixmap, QRectF(self.silah_pixmap.rect()))
        else:
            ressam.fillRect(-400, int(-400 + pitch_piksel), 800, 400, QColor("#0066cc"))
            ressam.fillRect(-400, int(0 + pitch_piksel), 800, 400, QColor("#8b4513"))

        ressam.setPen(QPen(QColor("#ffffff"), 2))
        ressam.drawLine(-150, int(0 + pitch_piksel), 150, int(0 + pitch_piksel)) 

        ressam.setFont(QFont("Arial", 8, QFont.Bold))
        for aci in range(-30, 35, 10):
            if aci == 0: continue
            y_pos = int(-aci * 2.5 + pitch_piksel)
            gen_cizgi = 40 if abs(aci) % 20 == 0 else 20
            ressam.setPen(QPen(QColor("#ffffff"), 1.5))
            ressam.drawLine(-gen_cizgi, y_pos, gen_cizgi, y_pos)
            ressam.drawText(gen_cizgi + 5, y_pos + 4, f"{aci}°")
            ressam.drawText(-gen_cizgi - 25, y_pos + 4, f"{aci}°")

        ressam.restore()

        ressam.setPen(QPen(QColor("#00ffcc"), 3))
        ressam.setBrush(Qt.NoBrush)
        ressam.drawEllipse(int(merkez_x - yaricap), int(merkez_y - yaricap), int(yaricap * 2), int(yaricap * 2))

        ressam.setPen(QPen(QColor("#ffd700"), 3))
        ressam.drawLine(int(merkez_x - 40), int(merkez_y), int(merkez_x - 10), int(merkez_y))
        ressam.drawLine(int(merkez_x + 10), int(merkez_y), int(merkez_x + 40), int(merkez_y))
        ressam.drawEllipse(int(merkez_x - 4), int(merkez_y - 4), 8, 8)
        
        ressam.setPen(QPen(QColor("#ffffff"), 1))
        ressam.setFont(QFont("Arial", 10, QFont.Bold))
        ressam.drawText(20, 30, "ATTITUDE (SUNİ UFUK)")


# ==============================================================================
# 3. SAĞ PANEL: KOORDİNELİ 3D TAKTİK RADAR (Nav2 + Mouse Kontrolü)
# ==============================================================================
class TaktikRadarEkrani(QFrame):
    hedef_belirlendi = pyqtSignal(float, float, float, float)  # ARTIK KULLANILMIYOR (bkz. rota_gonder_istendi) - geriye dönük uyumluluk için tutuldu
    rota_gonder_istendi = pyqtSignal(list)  # [(x, y, q_z, q_w), ...] - "ROTAYI GÖNDER" tıklanınca

    def __init__(self):
        super().__init__()
        self.setMinimumSize(450, 450)
        self.setStyleSheet("background-color: #0b0e14; border: 2px solid #1e2836; border-radius: 15px;")
        
        self.noktalar = []
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        # GÖRSEL YAW YUMUŞATMA (2026-09-01, "hala taktik lidar ksımında
        # kaymalar oluyor" - RViz'de yok - canlı ikinci kez bildirildi):
        # araç ikonu + canlı LiDAR bulutunun döndüğü açı (bkz. paintEvent,
        # ressam.rotate(-self.yaw_gorsel)) HAM /imu/data'dan geliyor (bkz.
        # HaritaYoneticisi.imu_guncelle) - /odom (pozisyon kaynağı, canlı
        # doğrulandı: bit-bit SABİT) FÜZYONLU/filtrelenmiş iken ham IMU
        # kuaterniyonu MEMS jiroskop/manyetometre gürültüsü TAŞIYABİLİR
        # (RViz'in araç modeli muhtemelen /odom TF zincirinden, yani
        # filtrelenmiş açıdan besleniyor - bu yüzden orada görünmüyor).
        # 15m menzilde 0.3°'lik bile HAM bir gürültü, en uzak LiDAR
        # noktalarında ~8cm'lik görünür bir "titreme/kayma" yaratıyor.
        # ÇÖZÜM: SADECE ÇİZİM için (navigasyon matematiğini - costmap/hedef
        # tıklama - ETKİLEMEZ, onlar ayrı/ham self.latest_yaw_rad kullanır,
        # bkz. Ros2GcsMotoru) üstel hareketli ortalama (EMA) ile yumuşatılmış
        # AYRI bir açı tutuluyor. Küçük kare-arası gürültüyü büker ama gerçek
        # dönüşlere (araç fiilen dönünce) birkaç örnek içinde (~100-150ms)
        # yetişir - RViz kadar pürüzsüz, gerçek manevrayı geciktirmeyecek kadar hızlı.
        self.yaw_gorsel = 0.0
        self._yaw_gorsel_ilk_mi = True
        # SABİT ÇERÇEVE (2026-09-01): eskiden 6m'lik dar bir görüş alanıydı
        # (LiDAR menzilini göstermeye yetiyordu, kamera zaten araca kilitli
        # olduğu için sorun değildi). Artık kamera SABİT (araç serbestçe
        # ekranda geziniyor) - kullanıcının sürdüğü/hedef verdiği rota tipik
        # olarak birkaç-birkaç on metre olabildiği için (bkz. RAMPA_HEDEF_
        # MESAFE_M=20m, tabela_etap_yoneticisi.py), gösterge alanı da buna
        # göre genişletildi. RViz'in nav2_view.rviz Orbit kamerasındaki
        # "Distance: 15" değeriyle paralel olsun diye 15m seçildi.
        self.maksimum_menzil = 15.0
        
        self.global_plan = []
        self.local_plan = []
        self.costmap_noktalari = []
        self.robot_x = 0.0
        self.robot_y = 0.0

        # GNSS - kullanicinin istegi uzerine taktik lidar ekraninda da
        # gorunsun diye (bkz. guncelle_gnss / paintEvent). None = henuz fix yok.
        self.gnss_enlem = None
        self.gnss_boylam = None

        # GPS KATKISI (2026-09-05, kullanıcı isteği: "gpsin konum
        # tahminine olan katkısı da yazılsın") - bkz. gps_katki_guncelle/
        # paintEvent. Boş = araçtan henüz hiç mesaj gelmedi.
        self.gps_katki_metni = ""

        self.mouse_start_pos = None
        self.mouse_current_pos = None
        self.hedef_ok_ciziliyor = False

        # FARE İLE YAKINLAŞTIRMA/KAYDIRMA (2026-09-01, kullanıcı isteği) -
        # SABİT ÇERÇEVE mimarisi (bkz. yukarısı) artık kamerayı araca
        # kilitlemediği için, uzak bir hedefe rota verildiğinde veya araç
        # uzaklaştığında görünüm dışına çıkabiliyor - RViz'deki gibi elle
        # yakınlaştırma/kaydırma gerekli. SOL TIK zaten hedef oku çizmek
        # için kullanılıyor (bkz. mousePressEvent) - kaydırma için SAĞ TIK
        # sürüklemesi kullanılıyor (haritacılık yazılımlarındaki yaygın
        # kural), böylece hiçbir çakışma olmuyor.
        self.pan_x = 0.0  # dünya (odom) metre - görünümün merkezlendiği nokta
        self.pan_y = 0.0
        self._pan_ciziliyor = False
        self._pan_baslangic_pos = None  # her mouseMoveEvent'te GÜNCELLENİR (artımlı sürükleme)

        # İMLEÇ KOORDİNAT OKUMASI (2026-09-01, kullanıcı isteği: "ben
        # görmediğim konumlarda nokta atacağım için gridin daha ayırt edici
        # olması gerekiyor hassas noktalar atabilmek için") - fare hareket
        # ederken (tıklamadan/sürüklemeden BAĞIMSIZ, bkz. __init__'teki
        # setMouseTracking) o anki dünya (/odom) konumu HUD'da canlı
        # gösterilir - tıklamadan ÖNCE tam olarak nereye basacağını görmek
        # RTK'lı hassas GPS ile bile ekrandan "göz kararı" tıklamaktan
        # kat kat daha güvenilir.
        self._imlec_dunya_x = None
        self._imlec_dunya_y = None

        # ÇOKLU NOKTA ROTASI (2026-09-01, kullanıcı isteği: "otonom gitmesi
        # için birkaç nokta vermemiz gerekiyor... birkaç işaret atabilelim")
        # - her sol-tık-sürükle-bırak artık DOĞRUDAN göndermek yerine bu
        # listeye EKLENİR (bkz. mouseReleaseEvent); "ROTAYI GÖNDER" butonuna
        # basılınca hepsi SIRAYLA (bkz. Ros2GcsMotoru.coklu_hedef_gonder)
        # gönderilir. Liste [(x, y, q_z, q_w), ...].
        self.hedef_listesi = []
        self._sag_panel_kur()
        self._liste_panelini_guncelle()

        self.setMouseTracking(True)
        self._son_hover_update_zamani = 0.0  # bkz. mouseMoveEvent'teki throttle notu

        # RENDER İSTEĞİ COALESCING (2026-09-05, canlı bildirildi: "arayüz
        # dondu, RTK verisi geç gidiyor, joystick bile geç gidiyor" -
        # crash_rapor_son.txt + [STALL] izleyicisi kök nedeni gösterdi):
        # guncelle_konum (/odom, ~59Hz!), guncelle_veri (LIDAR+IMU),
        # guncelle_gnss, guncelle_costmap, guncelle_global_plan,
        # guncelle_local_plan HEPSİ koşulsuz (sadece görünürlük kontrolüyle)
        # self.update() çağırıyordu - /odom TEK BAŞINA saniyede 59 kez
        # paintEvent'i tetikleyebiliyordu. RTK/GPS verisi ağ seviyesinde
        # SAĞLIKLI geliyor (canlı ölçüldü, ~10Hz) - "geç geliyor" izlenimi
        # aslında GUI thread'in bu AŞIRI SIK paintEvent tetiklenmesi altında
        # ekrana YANSITMASININ gecikmesiydi (turret_cmd_pub'daki QoS
        # sorunuyla AYNI sınıf: tek bir yavaş/meşgul GUI thread, kuyruktaki
        # TÜM güncellemeleri - GPS dahil, joystick/turret de dahil - eşit
        # şekilde geciktiriyordu). terminal_widget.py'deki AYNI coalescing
        # deseni (_render_istegi_beklemede bayrağı + debounce timer)
        # buraya da uygulandı: veri HER ZAMAN hemen güncellenir (state
        # ucuz), ama pahalı self.update() çağrısı en fazla ~30fps'e
        # (33ms) sınırlanır - bkz. _render_iste().
        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.timeout.connect(self._render_zamanlayici_calisti)
        self._render_istegi_beklemede = False

    def _render_iste(self):
        if not self.isVisible():
            return
        if not self._render_istegi_beklemede:
            self._render_istegi_beklemede = True
            self._render_timer.start(33)

    def _render_zamanlayici_calisti(self):
        self._render_istegi_beklemede = False
        self.update()

        # DOKUNMATİK EKRAN DESTEĞİ (2026-09-04, kullanıcı isteği: "scroll ile
        # yaptığım zoomu dokunmatik olarak da yapabilmek istiyorum... sağ
        # tık ile taşımayı da direkt elimle sürükleyince olması lazım") -
        # arayüz Jetson'da fiziksel fare/klavye YOK, dokunmatik ekran var
        # (bkz. proje notu). Sorunun kökü: Qt, dokunmayı VARSAYILAN olarak
        # SOL TIK'a çevirip sentetik mouse olayı üretir - bu da tek parmak
        # sürüklemesini "hedef oku çiz" (mousePressEvent/LeftButton) sanıp
        # YANLIŞLIKLA nokta ekliyordu (canlı bildirildi). WA_AcceptTouchEvents
        # ile GERÇEK QTouchEvent'ler alınıp event()'te ELLE işleniyor -
        # Qt bu widget için artık sentetik mouse üretmiyor, eski mouseX
        # metotları FARE (varsa) için hâlâ çalışmaya devam ediyor, ikisi
        # ÇAKIŞMIYOR. Davranış: TEK parmak KISA dokunuş (sürüklemeden) =
        # nokta ekle (varsayılan yönelimle - dokunarak yön çizmek pratik
        # değil); TEK parmak SÜRÜKLEME = kaydırma (eski SAĞ TIK sürüklemesiyle
        # BİREBİR AYNI matematik); İKİ parmak (pinch) = yakınlaştırma/
        # uzaklaştırma (eski fare tekerleğiyle AYNI his).
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self._dokunma_baslangic_pos = None
        self._dokunma_son_pos = None
        self._dokunma_hareket_etti = False
        self._dokunma_iki_parmak = False
        self._pinch_ilk_mesafe = None
        self._pinch_ilk_menzil = None
        # Bir dokunuşun "sürükleme" sayılması için gereken min. piksel -
        # bundan azı parmağın doğal titremesi/kayması sayılır, hâlâ TAP kabul edilir.
        self._DOKUNMA_SURUKLEME_ESIGI = 12

    def _sag_panel_kur(self):
        """Sağ tarafta rota noktalarının listesini (koordinat + araçtan
        mesafe) gösteren ve rota gönderme/temizleme butonlarını barındıran
        panel - kullanıcı isteği: 'sağ tarafta atılan işaretlerin mesafesi
        olsun'. resizeEvent'te sağa hizalanır (bkz. resizeEvent)."""
        self._panel = QWidget(self)
        self._panel.setStyleSheet(
            "QWidget { background-color: rgba(8, 12, 18, 235); "
            "border-left: 2px solid #1e2836; }"
        )
        duzen = QVBoxLayout(self._panel)
        duzen.setContentsMargins(8, 8, 8, 8)
        duzen.setSpacing(6)

        baslik = QLabel("ROTA NOKTALARI")
        baslik.setStyleSheet(
            "color:#00d4ff; font-weight:bold; font-size:12px; "
            "border:none; background:transparent;"
        )
        duzen.addWidget(baslik)

        self._nokta_listesi_w = QListWidget()
        self._nokta_listesi_w.setStyleSheet(
            "QListWidget { background-color:#0b0e14; color:#e6edf3; "
            "border:1px solid #1e2836; font-family:'DejaVu Sans Mono'; "
            "font-size:12px; }"
            # DOKUNMATIK (2026-09-04, kullanici: "silme dokunmatik ekranda
            # algilamiyor") - satir yuksekligi (padding) buyutuldu, tek
            # parmakla dogru satiri secmek kolaylassin diye.
            "QListWidget::item { padding:10px 4px; }"
            "QListWidget::item:selected { background-color:#1a3b5c; }"
        )
        self._nokta_listesi_w.setToolTip("Kaldırmak için: seçip aşağıdaki SİL'e basın (veya çift tıklayın)")
        self._nokta_listesi_w.itemDoubleClicked.connect(self._nokta_sil)
        duzen.addWidget(self._nokta_listesi_w, 1)

        # DOKUNMATIK SILME (2026-09-04, kullanici: "silme dokunmatik ekranda
        # algilamiyor") - cift-tikla/cift-dokunma touchscreen'de guvenilir
        # DEGIL (Qt'nin dokunma->fare senkronizasyonu tutarsiz). Bunun
        # yerine: satira TEK dokunusla sec, sonra bu BUYUK butona tek
        # dokunusla sil - iki AYRI tek-dokunus, cift-tiklamaya gerek yok.
        self._sil_btn = QPushButton("🗑 SEÇİLİ NOKTAYI SİL")
        self._sil_btn.setMinimumHeight(48)
        self._sil_btn.setStyleSheet(
            "QPushButton { background-color:#5c2e2e; color:white; "
            "font-weight:bold; font-size:12px; border-radius:6px; padding:8px; }"
            "QPushButton:hover { background-color:#763939; }"
        )
        self._sil_btn.clicked.connect(self._secili_noktayi_sil)
        duzen.addWidget(self._sil_btn)

        # GÖRECELİ NOKTA EKLEME (2026-09-02, kullanıcı isteği: "ilk noktayı
        # atınca sağ panelden ilk noktanın 6 metre sağına 7 metre soluna x
        # metre ilerisine diye noktalar ekleyebilelim") - haritaya tıklamak
        # yerine SAYI GİREREK, son eklenen noktaya (liste boşsa aracın o
        # anki konumuna) göre bir OFSET ile yeni nokta eklemeyi sağlar -
        # ekranda net görünmeyen ama bilinen bir mesafe/yönde bir nokta
        # gerektiğinde tıklamaktan çok daha hassas. YÖN SÖZLEŞMESİ v2
        # (2026-09-02, DÜZELTİLDİ - v1 EKRAN'ın sabit yönüne göreydi,
        # kullanıcı bunu istemedi: "araç hafif çapraz duruyorken yolda
        # ileri 3 metre at diyince aracın hizasında 3 metre atsın
        # istiyorum") - "ileri/sağ" artık ARACIN O ANKİ GERÇEK YÖNÜNE
        # (yaw) göre hesaplanır, ekranın sabit yukarısına göre DEĞİL (bkz.
        # _goreceli_nokta_ekle'deki dönüşüm). Hesap SADECE tıklama anındaki
        # yaw ile BİR KEZ yapılıp sabit bir dünya koordinatı üretir - araç
        # SONRADAN dönse bile o nokta yerinde kalır, sadece "ileri" o an
        # NEREYE bakıyorsa oraya göre hesaplanmış olur.
        self._referans_etiketi = QLabel("Referans: araç konumu")
        self._referans_etiketi.setStyleSheet(
            "color:#8b98a5; font-size:10px; border:none; background:transparent;"
        )
        duzen.addWidget(self._referans_etiketi)

        goreceli_izgara = QGridLayout()
        goreceli_izgara.setSpacing(3)

        def _spin_olustur():
            sb = QDoubleSpinBox()
            sb.setRange(-500.0, 500.0)
            sb.setSingleStep(0.5)
            sb.setDecimals(1)
            sb.setSuffix(" m")
            sb.setStyleSheet(
                "QDoubleSpinBox { background-color:#0b0e14; color:#e6edf3; "
                "border:1px solid #1e2836; padding:2px; }"
            )
            return sb

        etiket_ileri = QLabel("İleri(+)/Geri(-):")
        etiket_sag = QLabel("Sağ(+)/Sol(-):")
        for et in (etiket_ileri, etiket_sag):
            et.setStyleSheet("color:#e6edf3; font-size:10px; border:none; background:transparent;")

        self._ileri_spin = _spin_olustur()
        self._sag_spin = _spin_olustur()
        goreceli_izgara.addWidget(etiket_ileri, 0, 0)
        goreceli_izgara.addWidget(self._ileri_spin, 0, 1)
        goreceli_izgara.addWidget(etiket_sag, 1, 0)
        goreceli_izgara.addWidget(self._sag_spin, 1, 1)
        duzen.addLayout(goreceli_izgara)

        self._goreceli_ekle_btn = QPushButton("➕ Referansa Göre Ekle")
        self._goreceli_ekle_btn.setMinimumHeight(48)
        self._goreceli_ekle_btn.setStyleSheet(
            "QPushButton { background-color:#1a5276; color:white; "
            "font-weight:bold; font-size:12px; border-radius:6px; padding:8px; }"
            "QPushButton:hover { background-color:#21618c; }"
        )
        self._goreceli_ekle_btn.clicked.connect(self._goreceli_nokta_ekle)
        duzen.addWidget(self._goreceli_ekle_btn)

        self._durum_etiketi = QLabel("")
        self._durum_etiketi.setWordWrap(True)
        self._durum_etiketi.setStyleSheet(
            "color:#8b98a5; font-size:10px; border:none; background:transparent;"
        )
        duzen.addWidget(self._durum_etiketi)

        self._gonder_btn = QPushButton("🚀 ROTAYI GÖNDER")
        self._gonder_btn.setMinimumHeight(52)
        self._gonder_btn.setStyleSheet(
            "QPushButton { background-color:#00994d; color:white; "
            "font-weight:bold; font-size:13px; border-radius:6px; padding:8px; }"
            "QPushButton:hover { background-color:#00b359; }"
            "QPushButton:disabled { background-color:#2a3038; color:#5c6470; }"
        )
        self._gonder_btn.clicked.connect(self._rotayi_gonder_tikla)
        duzen.addWidget(self._gonder_btn)

        self._temizle_btn = QPushButton("✕ LİSTEYİ TEMİZLE")
        self._temizle_btn.setMinimumHeight(48)
        self._temizle_btn.setStyleSheet(
            "QPushButton { background-color:#992222; color:white; "
            "font-weight:bold; font-size:12px; border-radius:6px; padding:8px; }"
            "QPushButton:hover { background-color:#b32d2d; }"
        )
        self._temizle_btn.clicked.connect(self._listeyi_temizle)
        duzen.addWidget(self._temizle_btn)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # DOKUNMATIK (2026-09-04): butonlar buyudugu icin panel de 210->250
        # genisletildi, metinler sikismasin.
        genislik = 250
        self._panel.setGeometry(self.width() - genislik, 0, genislik, self.height())

    def guncelle_veri(self, noktalar, roll, pitch, yaw):
        self.noktalar = noktalar
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        # yaw_gorsel: bkz. __init__'teki not - SADECE çizim açısı, EMA ile
        # yumuşatılıyor. Açı sarmasını (ör. 179°→-179° "sıçraması" EMA'yı
        # yanlış yönde 358°'lik SAHTE bir dönüşle karıştırmasın diye) önce
        # [-180,180] aralığına normalize edilmiş FARK üzerinden ilerliyoruz.
        if self._yaw_gorsel_ilk_mi:
            self.yaw_gorsel = yaw
            self._yaw_gorsel_ilk_mi = False
        else:
            fark = (yaw - self.yaw_gorsel + 180.0) % 360.0 - 180.0
            self.yaw_gorsel += fark * 0.3
        # PERFORMANS: veri her zaman guncel tutulur (sekme degisince hemen
        # dogru goruntu cikar) ama pahali QPainter cizimi (self.update() ->
        # paintEvent) SADECE bu widget gercekten ekranda gorunurken tetiklenir.
        # Gorunmuyorken (baska bir ana sayfadayken) sürekli yeniden cizmek
        # bos yere CPU harcayip diger ekranlarda "kasma" yaratiyordu.
        # DÜZELTME (2026-09-05): self.update() DOĞRUDAN değil, _render_iste()
        # (coalescing, en fazla ~30fps) üzerinden - bkz. __init__'teki not.
        self._render_iste()

    def guncelle_global_plan(self, poses):
        self.global_plan = poses
        self._render_iste()

    def guncelle_local_plan(self, poses):
        self.local_plan = poses
        self._render_iste()

    def guncelle_costmap(self, noktalar):
        self.costmap_noktalari = noktalar
        self._render_iste()

    def gps_katki_guncelle(self, metin):
        # bkz. HaritaYoneticisi.__init__'teki 2026-09-05 notu - araç
        # tarafındaki konum_birlestirici.py'nin GPS düzeltmesi hakkında
        # yayınladığı insan-okunur durumu doğrudan gösterir.
        self.gps_katki_metni = metin
        self._render_iste()

    def guncelle_gnss(self, enlem, boylam):
        # Kullanicinin istegi: taktik lidar ekraninda da GNSS konumunu gor.
        self.gnss_enlem = enlem
        self.gnss_boylam = boylam
        self._render_iste()

    def guncelle_konum(self, x, y):
        self.robot_x = x
        self.robot_y = y
        # Ekran SABİT çerçevede olsa da (kamera aracı takip ETMİYOR, bkz.
        # _dunya_to_ekran) araç ikonunun kendisi odometri güncellendikçe
        # hareket ediyor - bu yüzden her /odom mesajında yeniden çizim
        # gerekiyor (eskiden robot_x/y sadece hata ayıklama yazısında
        # kullanıldığı için burada update() çağrısı yoktu). /odom ~59Hz
        # geldiği için (2026-09-05) DOĞRUDAN self.update() DEĞİL,
        # _render_iste() ile en fazla ~30fps'e sınırlanıyor.
        self._render_iste()

    def _gecerli_olcek(self):
        return min(self.width(), self.height()) / 2.0 / self.maksimum_menzil

    def _dunya_to_ekran(self, world_x, world_y):
        """Dünya (/odom) metre konumunu ekran pikseline çevirir - pan_x/y
        (bkz. __init__, sağ tık sürüklemesiyle değişir) görünümün o an
        HANGİ dünya noktasında merkezlendiğini belirler."""
        merkez_x, merkez_y = self.width() / 2.0, self.height() / 2.0
        olcek = self._gecerli_olcek()
        dx = world_x - self.pan_x
        dy = world_y - self.pan_y
        return merkez_x - dy * olcek, merkez_y - dx * olcek

    def _ekran_to_dunya(self, ekran_x, ekran_y):
        """_dunya_to_ekran'ın TERSİ - fare tıklamasını dünya konumuna çevirir."""
        merkez_x, merkez_y = self.width() / 2.0, self.height() / 2.0
        olcek = self._gecerli_olcek()
        dy = (merkez_x - ekran_x) / olcek
        dx = (merkez_y - ekran_y) / olcek
        return self.pan_x + dx, self.pan_y + dy

    def event(self, e):
        # bkz. __init__'teki "DOKUNMATİK EKRAN DESTEĞİ" notu.
        #
        # DUZELTME (2026-09-04, kullanici: "rota gönderme silme tuşları
        # ... dokunmatik ekranda algılamıyor", butonlar buyutulduktan
        # SONRA da devam etti - demek ki boyut degil, dokunma HIC
        # ULASMIYORDU): Qt'de WA_AcceptTouchEvents SADECE bu widget'ta
        # (TaktikRadarEkrani) acikken, ustundeki COCUK widget'lar (sag
        # paneldeki butonlar/liste - WA_AcceptTouchEvents'i KENDILERI
        # ayarlamiyor) normal tiklama gibi alt-widget hit-test'ine hic
        # girmiyor - TUM dokunmalar (panelin uzerindekiler DAHIL) burada
        # TaktikRadarEkrani.event()'e geliyor ve eskiden KOSULSUZ
        # `return True` ile YUTULUYORDU - butonlara HICBIR sekilde
        # ulasmiyordu (boyutlarinin bir onemi yoktu).
        #
        # Cozum: dokunma panel/buton alani UZERINDE BASLIYORSA burada
        # HIC ELE ALMIYORUZ (accept ETMIYORUZ, ignore) - Qt bunu otomatik
        # olarak SENTEZ FARE olayina cevirip normal hit-test ile dogru alt
        # widget'a (buton/liste) gonderiyor. Sadece CANVAS uzerinde
        # BASLAYAN dokunmalari (nokta ekleme/pan/pinch icin) biz aliyoruz.
        if e.type() == QEvent.TouchBegin:
            noktalar = e.touchPoints()
            if noktalar and noktalar[0].pos().x() >= self._panel.x():
                e.ignore()
                return False
            self._dokunma_baslat(e)
            return True
        if e.type() == QEvent.TouchUpdate:
            self._dokunma_guncelle(e)
            return True
        if e.type() in (QEvent.TouchEnd, QEvent.TouchCancel):
            self._dokunma_bitir(e)
            return True
        return super().event(e)

    def _dokunma_baslat(self, e):
        noktalar = e.touchPoints()
        if len(noktalar) >= 2:
            self._dokunma_iki_parmak = True
            p1, p2 = noktalar[0].pos(), noktalar[1].pos()
            self._pinch_ilk_mesafe = math.hypot(p1.x() - p2.x(), p1.y() - p2.y())
            self._pinch_ilk_menzil = self.maksimum_menzil
        elif len(noktalar) == 1:
            self._dokunma_iki_parmak = False
            self._dokunma_baslangic_pos = noktalar[0].pos()
            self._dokunma_son_pos = noktalar[0].pos()
            self._dokunma_hareket_etti = False

    def _dokunma_guncelle(self, e):
        noktalar = e.touchPoints()
        if len(noktalar) >= 2:
            # İKİ PARMAK = PİNCH YAKINLAŞTIRMA (eski fare tekerleğiyle AYNI
            # his) - parmaklar birbirinden UZAKLAŞIRSA (mesafe büyür)
            # yakınlaştır (maksimum_menzil küçülür), yaklaşırsa uzaklaştır.
            #
            # DÜZELTME: ikinci parmak neredeyse HİÇBİR ZAMAN ayrı bir
            # TouchBegin ile gelmiyor - Qt, ilk parmak zaten basılıyken
            # dokunan ikinci parmağı da TouchUpdate içine ekliyor (bkz.
            # Qt dokümantasyonu). Bu yüzden _pinch_ilk_mesafe SADECE
            # _dokunma_baslat'ta set edilseydi hep None kalır ve pinch hiç
            # çalışmazdı (canlı testte görülen tam olarak buydu). Çözüm:
            # 2 parmağa GEÇİŞ anını burada da yakalayıp referansı burada
            # başlat.
            p1, p2 = noktalar[0].pos(), noktalar[1].pos()
            mesafe = math.hypot(p1.x() - p2.x(), p1.y() - p2.y())
            if not self._dokunma_iki_parmak or not self._pinch_ilk_mesafe:
                self._dokunma_iki_parmak = True
                self._pinch_ilk_mesafe = mesafe
                self._pinch_ilk_menzil = self.maksimum_menzil
            elif mesafe > 1.0:
                oran = self._pinch_ilk_mesafe / mesafe
                self.maksimum_menzil = max(1.0, min(200.0, self._pinch_ilk_menzil * oran))
                self.update()
        elif len(noktalar) == 1 and not self._dokunma_iki_parmak:
            p = noktalar[0].pos()
            if self._dokunma_baslangic_pos is not None:
                toplam = math.hypot(p.x() - self._dokunma_baslangic_pos.x(),
                                     p.y() - self._dokunma_baslangic_pos.y())
                if toplam > self._DOKUNMA_SURUKLEME_ESIGI:
                    self._dokunma_hareket_etti = True
                if self._dokunma_hareket_etti:
                    # TEK PARMAK SÜRÜKLEME = KAYDIRMA (içerik parmağı takip
                    # eder). Canlı testte eski SAĞ-TIK formülüyle (aynen
                    # kopyalanmıştı) TERS yönde çalıştığı görüldü - bu
                    # dokunmatik panelin donanım/sürücü kalibrasyonunun
                    # fare imlecine göre ters model tarafında olduğunu
                    # gösteriyor; işareti burada çeviriyoruz (SADECE
                    # dokunmatik yol için - fare/sağ-tık sürüklemesine
                    # DOKUNULMADI).
                    olcek = self._gecerli_olcek()
                    delta_px = p.x() - self._dokunma_son_pos.x()
                    delta_py = p.y() - self._dokunma_son_pos.y()
                    self.pan_x += delta_py / olcek
                    self.pan_y += delta_px / olcek
                    self.update()
            self._dokunma_son_pos = p

    def _dokunma_bitir(self, e):
        if (not self._dokunma_iki_parmak and self._dokunma_baslangic_pos is not None
                and not self._dokunma_hareket_etti):
            # KISA DOKUNUŞ (sürüklemeden bırakıldı) = nokta ekle - dokunarak
            # yön/açı çizmek (eski sol-tık-sürükle-bırak jesti) pratik
            # olmadığından varsayılan yönelimle (q_z=0,q_w=1 - zaten
            # "GÖRECELİ EKLEME"deki AYNI varsayılan, gönderirken
            # _yonlendirilmis_rota() nasılsa gerçek yönü hesaplıyor).
            p = self._dokunma_baslangic_pos
            if p.x() < self._panel.x():
                hedef_x, hedef_y = self._ekran_to_dunya(p.x(), p.y())
                self.hedef_listesi.append((hedef_x, hedef_y, 0.0, 1.0))
                self._liste_panelini_guncelle()
                self.update()
        self._dokunma_baslangic_pos = None
        self._dokunma_son_pos = None
        self._dokunma_hareket_etti = False
        self._dokunma_iki_parmak = False
        self._pinch_ilk_mesafe = None

    def wheelEvent(self, event):
        # FARE TEKERLEĞİ İLE YAKINLAŞTIRMA (2026-09-01, kullanıcı isteği) -
        # uydu_harita_widget.py'deki AYNI kural/his (ArduPilot GCS'lerdeki
        # gibi). maksimum_menzil KÜÇÜLÜNCE yakınlaşır (nesneler büyür).
        carpan = 0.9 if event.angleDelta().y() > 0 else 1.1
        self.maksimum_menzil = max(1.0, min(200.0, self.maksimum_menzil * carpan))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_start_pos = event.pos()
            self.mouse_current_pos = event.pos()
            self.hedef_ok_ciziliyor = True
            self.update()
        elif event.button() == Qt.RightButton:
            # SAĞ TIK SÜRÜKLEMESİ İLE KAYDIRMA (2026-09-01, kullanıcı
            # isteği) - SOL TIK zaten hedef oku çizmek için kullanılıyor,
            # bu yüzden çakışmasın diye SAĞ TIK seçildi (haritacılık
            # yazılımlarındaki yaygın kural).
            self._pan_ciziliyor = True
            self._pan_baslangic_pos = event.pos()

    def mouseMoveEvent(self, event):
        # İMLEÇ KOORDİNAT OKUMASI (bkz. __init__'teki not) - sürükleme/
        # tıklama DURUMUNDAN BAĞIMSIZ, fare panel üzerinde DEĞİLKEN her
        # harekette güncellenir (hassas nokta atmadan ÖNCE nereye
        # basacağını görmek için).
        if event.pos().x() < self._panel.x():
            self._imlec_dunya_x, self._imlec_dunya_y = self._ekran_to_dunya(event.pos().x(), event.pos().y())
        else:
            self._imlec_dunya_x, self._imlec_dunya_y = None, None

        if self.hedef_ok_ciziliyor:
            self.mouse_current_pos = event.pos()
            self.update()
        elif self._pan_ciziliyor:
            olcek = self._gecerli_olcek()
            delta_px = event.pos().x() - self._pan_baslangic_pos.x()
            delta_py = event.pos().y() - self._pan_baslangic_pos.y()
            # Ekran formülünün (_dunya_to_ekran) TERSİ yönde: sürüklemeyi
            # dünya ofsetine çevirip pan_x/y'den ÇIKARIYORUZ (fareyi sağa
            # sürüklemek görünümü sola kaydırır - haritalardaki gibi).
            self.pan_x -= delta_py / olcek
            self.pan_y -= delta_px / olcek
            self._pan_baslangic_pos = event.pos()
            self.update()
        else:
            # DÜZELTME (2026-09-04, kullanıcı: "arayüzde kasmalar var" -
            # izole kamera testiyle sorunun ağ/araç DEĞİL, arayüz
            # tarafında olduğu doğrulandı) - burada HER TEK fare/dokunma
            # hareketinde (setMouseTracking AÇIK, Qt bunu ÇOK yüksek
            # sıklıkla gönderiyor) self.update() çağrılıyordu - bu da
            # koca radar ekranını (tüm grid çizgileri + noktalar +
            # araç ikonu) BAŞTAN yeniden çiziyordu, sadece imleç HUD
            # yazısını tazelemek için. Sadece HUD'daki imleç koordinat
            # yazısını tazelemek amacıyla saniyede onlarca/yüzlerce tam
            # ekran yeniden çizimi - GERÇEK "kasma" kaynaklarından biri
            # buydu. Artık bu (sürükleme/pan OLMAYAN, salt hover) dal en
            # fazla ~30Hz'e (33ms) throttle ediliyor - HUD yine akıcı
            # görünür ama gereksiz tekrar çizim önlenir.
            simdi_hover = time.time()
            if simdi_hover - self._son_hover_update_zamani < 0.033:
                return
            self._son_hover_update_zamani = simdi_hover
            self.update()

    def hedef_cizimini_iptal_et(self):
        # Kullanici istegi: rota/hedef oku cizerken ESC'ye basinca iptal
        # olsun - hedef GONDERILMEDEN suruklemeyi durdurur, onizleme okunu
        # temizler (bkz. main.py keyPressEvent, Qt.Key_Escape).
        if not self.hedef_ok_ciziliyor:
            return
        self.hedef_ok_ciziliyor = False
        self.mouse_start_pos = None
        self.mouse_current_pos = None
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.hedef_ok_ciziliyor:
            self.hedef_ok_ciziliyor = False

            start_x = self.mouse_start_pos.x()
            start_y = self.mouse_start_pos.y()
            end_x = event.pos().x()
            end_y = event.pos().y()

            # TAM SABİT ÇERÇEVE (2026-09-01, v2): _ekran_to_dunya artık
            # pan_x/y'yi de hesaba katıyor (kullanıcı sağ tıkla kaydırmış
            # olabilir) - tıklanan nokta HER ZAMAN doğru mutlak /odom
            # koordinatına denk gelir (RViz'de "2D Nav Goal" ile aynı mantık).
            hedef_ros_x, hedef_ros_y = self._ekran_to_dunya(start_x, start_y)

            dx = end_x - start_x
            dy = end_y - start_y

            hedef_yaw = math.atan2(-dx, -dy)

            q_z = math.sin(hedef_yaw / 2.0)
            q_w = math.cos(hedef_yaw / 2.0)

            # ÇOKLU NOKTA ROTASI (2026-09-01, kullanıcı isteği): artık
            # DOĞRUDAN göndermek yerine listeye EKLENİR - "ROTAYI GÖNDER"
            # butonuna basılana kadar araç hareket ETMEZ, kullanıcı
            # istediği kadar nokta ekleyebilir/çift tıklayarak silebilir.
            self.hedef_listesi.append((hedef_ros_x, hedef_ros_y, q_z, q_w))
            self._liste_panelini_guncelle()
            self.update()
        elif event.button() == Qt.RightButton and self._pan_ciziliyor:
            self._pan_ciziliyor = False
            self._pan_baslangic_pos = None

    def _liste_panelini_guncelle(self):
        """Sağ paneldeki listeyi self.hedef_listesi'nden yeniden çizer -
        her nokta için koordinat VE araçtan mesafe gösterilir (kullanıcı
        isteği: 'sağ tarafta atılan işaretlerin mesafesi olsun')."""
        self._nokta_listesi_w.clear()
        for i, (x, y, _q_z, _q_w) in enumerate(self.hedef_listesi, start=1):
            mesafe = math.hypot(x - self.robot_x, y - self.robot_y)
            self._nokta_listesi_w.addItem(
                QListWidgetItem(f"#{i}  X:{x:6.2f}  Y:{y:6.2f}  ({mesafe:5.1f}m)")
            )
        self._gonder_btn.setEnabled(bool(self.hedef_listesi))
        # GÖRECELİ EKLEME REFERANSI (bkz. _goreceli_nokta_ekle) - her zaman
        # SON eklenen nokta; liste boşsa aracın o anki konumu.
        if self.hedef_listesi:
            n = len(self.hedef_listesi)
            rx, ry, _, _ = self.hedef_listesi[-1]
            self._referans_etiketi.setText(f"Referans: Nokta #{n}  (X:{rx:.2f} Y:{ry:.2f})")
        else:
            self._referans_etiketi.setText(
                f"Referans: araç konumu (X:{self.robot_x:.2f} Y:{self.robot_y:.2f})")

    def _goreceli_nokta_ekle(self):
        """Sağ paneldeki İleri/Sağ kutularına girilen ofseti SON noktaya
        (liste boşsa araca) ekleyip yeni bir nokta oluşturur - kullanıcı
        isteği: '...ilk noktanın 6 metre sağına 7 metre soluna x metre
        ilerisine diye noktalar ekleyebilelim'.

        YÖN SÖZLEŞMESİ v2 (2026-09-02, DÜZELTİLDİ - v1 EKRAN'ın sabit
        yönüne göreydi, kullanıcı bunu istemedi: "araç hafif çapraz
        duruyorken yolda ileri 3 metre at diyince aracın hizasında 3
        metre atsın istiyorum" - yani "ileri/sağ" ARACIN O ANKİ GERÇEK
        YÖNÜNE (yaw) göre olmalı, ekranın sabit yukarısına göre DEĞİL).
        Hesaplama SADECE tıklama anındaki self.yaw ile BİR KEZ yapılıp
        SABİT bir dünya (/odom) koordinatı üretir (araç sonradan dönse
        bile bu nokta YERİNDE kalır - ekran zaten hiç dönmüyor, SABİT
        ÇERÇEVE). Kullanılan dönüşüm, `Ros2GcsMotoru._gecmis_mi`/
        `_dunya_to_arac`'ın (dünya→gövde) AYNI matrisinin TERSİ (gövde→
        dünya, dönüş matrisi olduğu için ters = devrik):
            ileri = dx*cos(yaw) + dy*sin(yaw)   (bkz. _dunya_to_arac)
            sol   = -dx*sin(yaw) + dy*cos(yaw)
        tersi:
            dx = ileri*cos(yaw) + sag*sin(yaw)  (sol=-sag, UI "Sağ(+)")
            dy = ileri*sin(yaw) - sag*cos(yaw)
        İzole matematik testiyle doğrulandı (yaw=0'da v1 ile AYNI sonucu
        verir - geriye dönük tutarlı; yaw=90°'de "ileri" dünya +Y'ye
        döner, vb.)."""
        if self.hedef_listesi:
            ref_x, ref_y, _, _ = self.hedef_listesi[-1]
        else:
            ref_x, ref_y = self.robot_x, self.robot_y

        ileri_m = self._ileri_spin.value()
        sag_m = self._sag_spin.value()
        yaw_rad = math.radians(self.yaw)
        dx = ileri_m * math.cos(yaw_rad) + sag_m * math.sin(yaw_rad)
        dy = ileri_m * math.sin(yaw_rad) - sag_m * math.cos(yaw_rad)
        yeni_x = ref_x + dx
        yeni_y = ref_y + dy

        self.hedef_listesi.append((yeni_x, yeni_y, 0.0, 1.0))
        self._liste_panelini_guncelle()
        self._ileri_spin.setValue(0.0)
        self._sag_spin.setValue(0.0)
        self.update()

    def _nokta_sil(self, item):
        satir = self._nokta_listesi_w.row(item)
        if 0 <= satir < len(self.hedef_listesi):
            del self.hedef_listesi[satir]
            self._liste_panelini_guncelle()
            self.update()

    def _secili_noktayi_sil(self):
        # DOKUNMATIK SILME (bkz. _sag_panel_kur'daki not) - listede secili
        # (tek dokunusla secilmis) satiri siler. currentRow() secim yoksa
        # -1 doner, _nokta_sil zaten bunu 0<=satir kontroluyle yok sayar.
        satir = self._nokta_listesi_w.currentRow()
        if 0 <= satir < len(self.hedef_listesi):
            del self.hedef_listesi[satir]
            self._liste_panelini_guncelle()
            self.update()

    def _listeyi_temizle(self):
        self.hedef_listesi = []
        self._liste_panelini_guncelle()
        self._durum_etiketi.setText("")
        self.update()

    def _rotayi_gonder_tikla(self):
        if not self.hedef_listesi:
            return
        self.rota_gonder_istendi.emit(self._yonlendirilmis_rota())
        self._gonder_btn.setEnabled(False)
        self._temizle_btn.setEnabled(False)

    def _yonlendirilmis_rota(self):
        """DÜZELTME (2026-09-02, canlı bildirildi: "araç 1. noktaya geldi
        ters tarafa döndü durdu sonra uzun bir dönüş yaptı, öyle olmasın") -
        göreceli panelden eklenen noktaların hepsi varsayılan/rastgele bir
        yönelimle (q_z=0, q_w=1) gönderiliyordu - araç o noktaya HANGİ
        açıyla varırsa varsın diye bir hedefti, bu da bazen SIRADAKİ
        noktanın TAM TERSİ bir yöne bakarak durup sonra uzun bir dönüşle
        toparlanmasına yol açıyordu. Artık GÖNDERMEDEN HEMEN ÖNCE (tüm
        noktaların nihai konumu zaten belli olduğu için tek seferde
        hesaplanabiliyor) her ara nokta SIRADAKİ noktaya bakacak şekilde
        yönlendiriliyor - varış anında zaten doğru yöne dönük olur, ters
        dönüşe gerek kalmaz. SON nokta için "sıradaki" yok - bir önceki
        bacağın yönünü DÜZ SÜRDÜRÜYORMUŞ gibi (extrapole) yönlendirilir
        (rastgele 0 yerine). Açı formülü `mouseReleaseEvent`'teki sürükleme
        jestiyle BİREBİR AYNI (`atan2(dy_world, dx_world)` - izole
        matematik testiyle doğrulandı, ekran-pikseli tabanlı sürükleme
        formülüyle her zaman özdeş sonuç verir)."""
        n = len(self.hedef_listesi)
        yonlendirilmis = []
        for i in range(n):
            x, y, eski_qz, eski_qw = self.hedef_listesi[i]
            if i < n - 1:
                hedef_x, hedef_y, _, _ = self.hedef_listesi[i + 1]
            elif n >= 2:
                onceki_x, onceki_y, _, _ = self.hedef_listesi[i - 1]
                hedef_x, hedef_y = x + (x - onceki_x), y + (y - onceki_y)
            else:
                yonlendirilmis.append((x, y, eski_qz, eski_qw))
                continue
            aci = math.atan2(hedef_y - y, hedef_x - x)
            yonlendirilmis.append((x, y, math.sin(aci / 2.0), math.cos(aci / 2.0)))
        return yonlendirilmis

    def rota_durum_guncelle(self, metin):
        """Ros2GcsMotoru.rota_durum_sinyali'ne bağlanır (bkz.
        HaritaYoneticisi) - gönderim ilerlemesini/sonucunu panelde gösterir."""
        self._durum_etiketi.setText(metin)
        if "TAMAMLANDI" in metin or "DURDURULDU" in metin:
            # Rota bitti (başarılı ya da durduruldu) - gönderilenler artık
            # listede tutulmaya gerek yok, yeni bir rota için temizle.
            self.hedef_listesi = []
            self._liste_panelini_guncelle()
            self._temizle_btn.setEnabled(True)
            self.update()

    def paintEvent(self, event):
        # DÜZELTME (2026-09-05, segfault kökeni): QPainter try/finally ile
        # sarmalanıyor - bkz. SuniUfukEkrani.paintEvent üstündeki not ve
        # uydu_harita_widget.py'deki gerçek SIGSEGV kaydı. Bu ekran özellikle
        # riskli: harici LIDAR/costmap/waypoint verisi üzerinde döngü kuruyor,
        # herhangi biri beklenmedik şekil/None gelirse istisna atabilir.
        #
        # GEÇİCİ PROFİLLEME (2026-09-06, kullanıcı: "lidar ekranında çok
        # gecikme var, bazen iyi çalışıyor ardından kötüleşebiliyor") -
        # "zamanla kötüleşme" birikmeli bir soruna işaret ediyor; hangi
        # bölümün ne kadar sürdüğünü VE zamanla artıp artmadığını ölçmek
        # için her 60 karede bir özet basılıyor (bkz. _profil_raporla).
        _t0 = time.perf_counter()
        ressam = QPainter(self)
        try:
            ressam.setRenderHint(QPainter.Antialiasing)
            self._paintEvent_govde(ressam)
        finally:
            ressam.end()
        self._profil_kaydet((time.perf_counter() - _t0) * 1000.0)

    def _profil_kaydet(self, sure_ms):
        gecmis = getattr(self, '_profil_gecmis', None)
        if gecmis is None:
            gecmis = self._profil_gecmis = []
        gecmis.append(sure_ms)
        if len(gecmis) < 60:
            return
        self._profil_gecmis = []
        ort = sum(gecmis) / len(gecmis)
        en_kotu = max(gecmis)
        # veri boyutları da basılıyor: zamanla büyüyen bir liste varsa
        # (birikme) burada net görünür.
        bolum = getattr(self, '_son_bolum', {})
        bolum_str = " ".join(f"{k}={v:.1f}" for k, v in bolum.items())
        print(f"[RADAR-PROFIL] paintEvent ort={ort:.1f}ms en_kötü={en_kotu:.1f}ms | "
              f"BÖLÜMLER(ms): {bolum_str} | "
              f"lidar={len(self.noktalar)} costmap={len(self.costmap_noktalari)} "
              f"gplan={len(self.global_plan) if self.global_plan else 0} "
              f"lplan={len(self.local_plan) if self.local_plan else 0} "
              f"hedef={len(self.hedef_listesi)} menzil={self.maksimum_menzil:.0f}m")

    def _zemin_pixmap_al(self, g, y, olcek, merkez_x, merkez_y):
        """Arka plan gradyanı + grid + merkez eksenlerini ÖNBELLEKLENMİŞ bir
        QPixmap olarak döndürür.

        PERFORMANS (2026-09-06, kullanıcı: "lidar ekranında çok gecikme var,
        bazen iyi çalışıyor ardından kötüleşebiliyor") - canlı bölüm bazlı
        profillemeyle ÖLÇÜLDÜ: paintEvent'in ~26ms'sinin ~14ms'si SADECE
        arka plan (QRadialGradient ile TÜM ekranı doldurma, ~5.6ms) ve grid
        (~33 iterasyon, her birinde YENİ QPen + 2 drawLine, ~8.4ms) çizimine
        gidiyordu - VE BUNLAR HER KAREDE (saniyede ~30 kez) yeniden
        çiziliyordu. Oysa ikisi de SADECE pencere boyutu, pan (sürükleme)
        veya zoom değişince değişir - yani kullanıcı etkileşimi anlarında,
        saniyede 30 kez değil. Artık tek bir pixmap'e bir kez çizilip
        önbelleğe alınıyor, her karede sadece TEK bir drawPixmap (bellek
        kopyalama - kıyasla neredeyse bedava) yapılıyor.
        """
        anahtar = (g, y, round(self.pan_x, 4), round(self.pan_y, 4),
                   round(self.maksimum_menzil, 4))
        if getattr(self, '_zemin_anahtar', None) == anahtar:
            onbellek = getattr(self, '_zemin_pixmap', None)
            if onbellek is not None:
                return onbellek

        pix = QPixmap(g, y)
        p = QPainter(pix)
        try:
            p.setRenderHint(QPainter.Antialiasing)
            grad = QRadialGradient(merkez_x, merkez_y, min(merkez_x, merkez_y))
            grad.setColorAt(0.0, QColor("#0f1c2b"))
            grad.setColorAt(1.0, QColor("#0b0e14"))
            p.fillRect(0, 0, g, y, grad)

            # GRID (bkz. aşağıdaki eski yorumun tamamı) - artık burada,
            # önbelleğe çiziliyor. Ek optimizasyon: QPen nesneleri döngü
            # İÇİNDE değil, döngü DIŞINDA bir kez oluşturuluyor (eskiden
            # her iterasyonda yeni bir QPen+QColor yaratılıyordu).
            grid_araligi_m = 1.0
            while grid_araligi_m * olcek < 6.0:
                grid_araligi_m *= 5.0 if grid_araligi_m < 5.0 else 2.0
            ilk_cizgi_m = -(int(self.maksimum_menzil / grid_araligi_m) + 1) * grid_araligi_m
            hucre_sayisi = int(2 * self.maksimum_menzil / grid_araligi_m) + 3
            kalem_ana = QPen(QColor("#2a3a4a"), 1)
            kalem_ince = QPen(QColor("#161f28"), 1)
            for i in range(hucre_sayisi):
                ofset_m = ilk_cizgi_m + i * grid_araligi_m
                hucre_no = round(ofset_m / grid_araligi_m)
                p.setPen(kalem_ana if (hucre_no % 5 == 0) else kalem_ince)
                dikey_x, _ = self._dunya_to_ekran(self.pan_x, self.pan_y + ofset_m)
                _, yatay_y = self._dunya_to_ekran(self.pan_x + ofset_m, self.pan_y)
                if 0 <= dikey_x <= g:
                    p.drawLine(int(dikey_x), 0, int(dikey_x), y)
                if 0 <= yatay_y <= y:
                    p.drawLine(0, int(yatay_y), g, int(yatay_y))

            p.setPen(QPen(QColor("#3a5a7a"), 1))
            p.drawLine(int(merkez_x), 0, int(merkez_x), y)
            p.drawLine(0, int(merkez_y), g, int(merkez_y))
        finally:
            p.end()

        self._zemin_anahtar = anahtar
        self._zemin_pixmap = pix
        return pix

    def _paintEvent_govde(self, ressam):
        _b = time.perf_counter()
        _bolum = {}
        g = self.width()
        y = self.height()
        merkez_x, merkez_y = g / 2, y / 2
        olcek = min(merkez_x, merkez_y) / self.maksimum_menzil

        # Arka plan + grid + merkez eksenleri: önbellekten TEK çağrıyla
        # (bkz. _zemin_pixmap_al - ölçülen ~14ms buradan kurtarıldı).
        ressam.drawPixmap(0, 0, self._zemin_pixmap_al(g, y, olcek, merkez_x, merkez_y))
        _bolum['zemin'] = (time.perf_counter() - _b) * 1000; _b = time.perf_counter()
        self._son_bolum = _bolum

        # SABİT ÇERÇEVE - TAM RViz DAVRANIŞI (2026-09-01, v2 - kullanıcı
        # isteği: "map frame odom olsun, rviz gibi olsun, kamera aracı takip
        # ETMESİN"). v1'de kamera hâlâ aracın konumunu takip ediyordu (sadece
        # döndürmüyordu) - bu YETERSİZ bulundu ("hala rota üzerinde araç
        # ilerlemiyor", canlı bildirildi): araç ekranda HER ZAMAN aynı
        # noktada (merkez) kaldığı için, gerçekten "rota üzerinde ilerlediği"
        # hissi hâlâ oluşmuyordu. v2: EKRANIN KENDİSİ artık dünya (/odom)
        # çerçevesine TAMAMEN SABİT - kamera ARTIK HİÇ ÖTELEMİYOR/DÖNDÜRMÜYOR.
        # odom'un kendi orijini (0,0) = aracın açılıştaki/ilk konumu (ROS
        # kuralı) VARSAYILAN olarak ekran merkezine denk gelir - araç oradan
        # ne kadar uzaklaşırsa uzaklaşsın, ekrandaki konumu da GERÇEKTEN o
        # kadar kayar (RViz'de "Fixed Frame: odom" ile birebir aynı - bkz.
        # nav2_view.rviz). Kullanıcı SAĞ TIK sürükleyerek görünümü kaydırıp
        # (bkz. mousePressEvent/self.pan_x/y) veya fare tekerleğiyle
        # yakınlaştırıp (bkz. wheelEvent) aracı/rotayı görüş alanına
        # getirebilir - RViz'deki elle kamera kontrolünün aynısı.
        # (bkz. _dunya_to_ekran/_ekran_to_dunya metodları)
        arac_ekran_x, arac_ekran_y = self._dunya_to_ekran(self.robot_x, self.robot_y)

        # GRID: artık burada DEĞİL - önbelleklenmiş zemin pixmap'ine taşındı
        # (bkz. _zemin_pixmap_al). Grid dünya eksenlerine hizalı, tüm görünür
        # alanı kaplayan ince çizgilerden oluşur (1m aralık, her 5 hücrede
        # bir daha parlak "ana" hat) ve SADECE pan/zoom/boyut değişince
        # yeniden üretilir - her karede yeniden çizilmesi ölçülen ~8.4ms
        # israftı.

        # Mesafe etiketleri: eskiden aracı çevreleyen halka/karelerin
        # üzerindeydi - artık düz grid'de onun yerine aracın SAĞINDAN
        # geçen dikey eksende, her 5 metrede bir okunabilir etiket var.
        # (Etiketler araç konumuna bağlı olduğu için zemin önbelleğine
        # ALINAMAZ - araç hareket ettikçe yerleri değişir. setPen döngü
        # DIŞINA alındı: her iterasyonda yeni QPen/QColor yaratmak gereksizdi.)
        etiket_araligi = 5
        ressam.setFont(QFont("Arial", 8))
        ressam.setPen(QPen(QColor("#0088cc"), 1))
        for m in range(etiket_araligi, int(self.maksimum_menzil) + 1, etiket_araligi):
            ressam.drawText(int(arac_ekran_x + 5), int(arac_ekran_y - m * olcek + 12), f"{m}m")
            ressam.drawText(int(arac_ekran_x + 5), int(arac_ekran_y + m * olcek + 12), f"{m}m")
        # NOT: merkez eksen çizgileri artık zemin önbelleğinde (yukarıda
        # tekrar çizilmiyor - eskiden burada da vardı, duplikasyondu).
        _bolum['etiket'] = (time.perf_counter() - _b) * 1000; _b = time.perf_counter()

        if self.global_plan:
            ressam.setPen(QPen(QColor(0, 150, 255, 200), 3, Qt.DashLine))
            path_polygon = QPolygonF()
            for world_x, world_y in self.global_plan:
                p_x, p_y = self._dunya_to_ekran(world_x, world_y)
                path_polygon.append(QPointF(p_x, p_y))
            ressam.drawPolyline(path_polygon)

        if self.local_plan:
            ressam.setPen(QPen(QColor(255, 200, 0, 255), 4))
            path_polygon = QPolygonF()
            for world_x, world_y in self.local_plan:
                p_x, p_y = self._dunya_to_ekran(world_x, world_y)
                path_polygon.append(QPointF(p_x, p_y))
            ressam.drawPolyline(path_polygon)

        # Costmap "engel izi" (RViz'deki gibi kalıcı/sönümlü iz) - canlı lidar
        # noktalarının ALTINDA, soluk turuncu olarak çiziliyor.
        if self.costmap_noktalari:
            # PERFORMANS (2026-09-05, kullanıcı: "lidar ekranında çok fazla
            # komut verisi gecikiyor") - eskiden HER NOKTA için AYRI bir
            # drawRect() çağrısı vardı (300'e kadar) - her ayrı native Qt
            # çağrısı PyQt/SIP binding geçişi (Python<->C++) taşıyor, bu
            # yazılımsal (GPU hızlandırmasız) render'da yüzlerce noktada
            # TOPLAMDA büyük maliyete dönüşüyordu. Artık TÜM noktalar TEK
            # BİR QPolygonF'e toplanıp TEK BİR drawPoints() çağrısıyla
            # çiziliyor - Qt'nin native tarafı diziyi TEK SEFERDE
            # rasterize ediyor, 300 ayrı çağrı yerine 1 çağrı. Görsel fark:
            # eskiden 4x4 KARE, şimdi RoundCap ile yuvarlak nokta (aynı
            # boyutta, gözle neredeyse ayırt edilemez).
            # PERFORMANS (2026-09-06): eskiden döngü içinde her nokta için
            # self._dunya_to_ekran() çağrılıyordu - o metod HER çağrıda
            # _gecerli_olcek()'i (min/width()/height() hesabı) YENİDEN
            # yapıyordu, yani ~300 nokta için 300 gereksiz hesap + 300
            # Python metot çağrısı. Ölçek/merkez döngü DIŞINDA bir kez
            # hesaplanıp dönüşüm satır içine alındı (aynı matematik).
            ressam.setRenderHint(QPainter.Antialiasing, False)
            costmap_dizisi = QPolygonF()
            _pan_x, _pan_y = self.pan_x, self.pan_y
            for world_x, world_y in self.costmap_noktalari:
                costmap_dizisi.append(QPointF(
                    merkez_x - (world_y - _pan_y) * olcek,
                    merkez_y - (world_x - _pan_x) * olcek))
            ressam.setPen(QPen(QColor(255, 140, 0, 90), 4, Qt.SolidLine, Qt.RoundCap))
            ressam.drawPoints(costmap_dizisi)
            ressam.setRenderHint(QPainter.Antialiasing, True)
        _bolum['costmap'] = (time.perf_counter() - _b) * 1000; _b = time.perf_counter()

        # ÇOKLU NOKTA ROTASI: bekleyen (henüz gönderilmemiş) noktalar
        # numaralı sarı işaretler + aralarında ince bağlantı çizgisiyle
        # gösterilir (kullanıcı isteği: "birkaç işaret atabilelim").
        if self.hedef_listesi:
            ressam.setPen(QPen(QColor(255, 215, 0, 160), 2, Qt.DotLine))
            onceki_ekran = None
            for x, y, _q_z, _q_w in self.hedef_listesi:
                p_x, p_y = self._dunya_to_ekran(x, y)
                if onceki_ekran is not None:
                    ressam.drawLine(QPointF(*onceki_ekran), QPointF(p_x, p_y))
                onceki_ekran = (p_x, p_y)
            # YÖNELİM OKU (2026-09-02, kullanıcı isteği: "sadece noktayı
            # değil küçük bir ok şeklinde yönelimini de göster") -
            # _rotayi_gonder_tikla'da GERÇEKTEN GÖNDERİLECEK olan AYNI
            # zincirleme yön (bkz. _yonlendirilmis_rota - her nokta
            # SIRADAKİNE bakar) burada GÖNDERMEDEN ÖNCE önizlenir, ki
            # kullanıcı her noktanın hangi yöne bakacağını gözle
            # doğrulayabilsin (panelden eklenen noktaların ham/kayıtlı
            # q_z=0 değeri DEĞİL, gönderim anında hesaplanacak GERÇEK yön
            # gösteriliyor).
            ok_uzunluk_m = 1.5
            for i, (x, y, q_z, q_w) in enumerate(self._yonlendirilmis_rota(), start=1):
                p_x, p_y = self._dunya_to_ekran(x, y)

                aci_rad = 2.0 * math.atan2(q_z, q_w)
                uc_dunya_x = x + ok_uzunluk_m * math.cos(aci_rad)
                uc_dunya_y = y + ok_uzunluk_m * math.sin(aci_rad)
                uc_x, uc_y = self._dunya_to_ekran(uc_dunya_x, uc_dunya_y)

                ressam.setPen(QPen(QColor("#00ff88"), 2.5))
                ressam.drawLine(QPointF(p_x, p_y), QPointF(uc_x, uc_y))
                ok_aci = math.atan2(uc_y - p_y, uc_x - p_x)
                ok_boy = 7
                ressam.setBrush(QColor("#00ff88"))
                ressam.setPen(Qt.NoPen)
                ressam.drawPolygon(QPolygonF([
                    QPointF(uc_x, uc_y),
                    QPointF(uc_x - ok_boy * math.cos(ok_aci - math.pi / 6),
                             uc_y - ok_boy * math.sin(ok_aci - math.pi / 6)),
                    QPointF(uc_x - ok_boy * math.cos(ok_aci + math.pi / 6),
                             uc_y - ok_boy * math.sin(ok_aci + math.pi / 6)),
                ]))

                ressam.setBrush(QColor("#ffd700"))
                ressam.setPen(QPen(QColor("#000000"), 1.5))
                ressam.drawEllipse(QPointF(p_x, p_y), 10, 10)
                ressam.setPen(QPen(QColor("#000000"), 1))
                ressam.setFont(QFont("Arial", 9, QFont.Bold))
                ressam.drawText(QRectF(p_x - 10, p_y - 10, 20, 20), Qt.AlignCenter, str(i))

        # Araç ikonu: artık ekranın GERÇEK konumuna (arac_ekran_x/y) çiziliyor
        # (merkez DEĞİL - kamera artık takip etmiyor, bkz. yukarısı) VE
        # KENDİSİ self.yaw kadar dönüyor - RViz'deki "araç modeli sabit
        # harita üzerinde gerçekten hareket edip döner" davranışının aynısı.
        ressam.save()
        ressam.translate(arac_ekran_x, arac_ekran_y)
        # DÜZELTME (2026-09-01, v5): ham self.yaw YERİNE EMA ile yumuşatılmış
        # self.yaw_gorsel kullanılıyor - bkz. __init__/guncelle_veri notları.
        ressam.rotate(-self.yaw_gorsel)

        # LiDAR: gövde-göreceli (x_m=ileri, y_m=sol) - KRİTİK: bu artık
        # AYRI bir dünya-dönüşümü YAPMIYOR, doğrudan ARACIN KENDİ
        # translate+rotate bloğunun İÇİNDE, Qt'nin TEK BİR transform
        # matrisiyle çiziliyor (eski/orijinal koddaki AYNI teknik). Önceki
        # deneme (ayrı bir manuel sin/cos ile "dünya konumuna" çevirip
        # SONRA ikonu AYRI bir transformla çizmek) canlı olarak GERÇEK bir
        # titreme/oynama sorununa yol açtı ("araç sabit duruyor ama Taktik
        # LiDAR'da sürekli oynama oluyor, RViz'de yok" - canlı bildirildi):
        # IMU her güncellendiğinde (~60Hz throttle) self.yaw ÇOK KÜÇÜK de
        # olsa gürültü içeriyor - iki AYRI transform (biri lidar için biri
        # ikon için) kullanmak, aralarında hiçbir GERÇEK fark olmasa bile
        # kayan noktalı yuvarlama farkları ve zamanlama nedeniyle LiDAR
        # bulutunun sabit costmap ızgarasına göre GÖRECELİ olarak titremesine
        # yol açtı - eskiden hem lidar hem ikon AYNI TEK transform'u
        # paylaştığı için bu görünmüyordu. Artık yine paylaşıyorlar - lidar
        # ve ikon HER ZAMAN birebir aynı anda, birebir aynı açıyla döner,
        # aralarında GÖRECELİ titreme MATEMATİKSEL OLARAK imkansız.
        # NOKTA BOYUTU (2026-09-01, canlı bildirilen "hala kaymalar oluyor"
        # şikayeti araştırılırken bulundu): RViz'in KENDİ LaserScan ayarı
        # ("nav2_view.rviz") noktaları SABİT 0.05m (5cm) dünya boyutunda
        # çiziyor - burada ise SABİT 6 PİKSEL kullanılıyordu. Görüş alanı
        # 15m'ye çıkarılınca (bkz. maksimum_menzil notu) bu, gerçek dünya
        # ölçeğinde ~0.4m'ye denk geliyordu - RViz'dekinin ~8 KATI BÜYÜK.
        # Herhangi bir LiDAR'ın normal/beklenen ölçüm gürültüsü (birkaç cm,
        # TÜM lazer sensörlerinde vardır, RViz'in küçük noktalarında görünmez)
        # bu kadar büyük noktalarda oransal olarak ÇOK daha belirgin/rahatsız
        # edici bir "kayma" gibi görünüyordu. Artık RViz'le AYNI 5cm dünya
        # boyutunu kullanıyor (min 1.5px - çok uzak zoom'da tamamen kaybolmasın).
        nokta_yaricap = max(1.5, 0.025 * olcek)  # 0.05m çap = 0.025m yarıçap
        # PERFORMANS (2026-09-05, kullanıcı: "lidar ekranında çok fazla
        # komut verisi gecikiyor") - eskiden HER LIDAR NOKTASI için AYRI
        # bir drawEllipse() çağrısı vardı (300'e kadar, PAINT EVENT'İN EN
        # PAHALI kısmıydı - STALL izleyicisi bu satırı defalarca 400-900ms+
        # takılı yakaladı). AYNI costmap düzeltmesindeki mantıkla, TÜM
        # noktalar TEK BİR QPolygonF'e toplanıp TEK BİR drawPoints()
        # çağrısıyla çiziliyor - 300 ayrı native Qt çağrısı yerine 1 çağrı.
        # Antialiasing bu kadar küçük (1.5-birkaç px) noktalarda gözle
        # neredeyse fark edilmiyor ama yazılımsal çizimde maliyetli, bu
        # yüzden sadece bu çizim için kapatılıp hemen geri açılıyor.
        ressam.setRenderHint(QPainter.Antialiasing, False)
        lidar_dizisi = QPolygonF()
        for x_m, y_m in self.noktalar:
            p_x = -y_m * olcek
            p_y = -x_m * olcek
            lidar_dizisi.append(QPointF(p_x, p_y))
        ressam.setPen(QPen(QColor("#ff0033"), nokta_yaricap * 2, Qt.SolidLine, Qt.RoundCap))
        ressam.drawPoints(lidar_dizisi)
        ressam.setRenderHint(QPainter.Antialiasing, True)
        _bolum['lidar'] = (time.perf_counter() - _b) * 1000; _b = time.perf_counter()

        uzama_3d = int(-self.pitch * 0.7)
        golge_buyukluk = max(0, int(self.pitch * 0.5))

        if golge_buyukluk > 0:
            ressam.setBrush(QColor(0, 0, 0, 150))
            ressam.setPen(Qt.NoPen)
            ressam.drawRoundedRect(-16, -18 + uzama_3d + golge_buyukluk, 32, 44, 8, 8)

        ressam.save()
        ressam.rotate(self.roll * 0.3)

        koni_grad = QRadialGradient(0, 0, 140)
        koni_grad.setColorAt(0.0, QColor(0, 255, 204, 70))
        koni_grad.setColorAt(1.0, QColor(0, 255, 204, 0))
        ressam.setBrush(koni_grad)
        ressam.setPen(Qt.NoPen)
        ressam.drawPie(-140, -140 + uzama_3d, 280, 280, 70 * 16, 40 * 16) 

        ressam.setBrush(QColor("#0a0d12"))
        ressam.setPen(QPen(QColor("#00ffcc"), 1.5))
        ressam.drawRoundedRect(-22, -20, 8, 40, 2, 2)
        ressam.drawRoundedRect(14, -20, 8, 40, 2, 2)

        sasi_grad = QLinearGradient(0, -25, 0, 25)
        sasi_grad.setColorAt(0.0, QColor("#223548"))
        sasi_grad.setColorAt(1.0, QColor("#111a24"))
        ressam.setBrush(sasi_grad)
        ressam.setPen(QPen(QColor("#00bfff"), 2))
        ressam.drawRoundedRect(-14, -22 + uzama_3d, 28, 44, 6, 6)

        ressam.setBrush(QColor("#ffd700"))
        ressam.setPen(Qt.NoPen)
        ok = QPolygon([QPoint(0, -36 + uzama_3d), QPoint(-9, -18 + uzama_3d), 
                       QPoint(0, -22 + uzama_3d), QPoint(9, -18 + uzama_3d)])
        ressam.drawPolygon(ok)

        ressam.setBrush(QColor("#ff0033"))
        ressam.drawEllipse(-6, -6 + uzama_3d, 12, 12)
        ressam.setBrush(QColor("#ffffff"))
        ressam.drawEllipse(-2, -2 + uzama_3d, 4, 4)

        ressam.restore()
        ressam.restore()

        # Araç resminin altında canlı yeşil açı/konum yazısı (eski masaüstü kodundaki gibi)
        ressam.setPen(QPen(QColor("#00ff00"), 1))
        ressam.setFont(QFont("Arial", 10, QFont.Bold))
        ressam.drawText(int(merkez_x - 70), int(merkez_y + 50),
                         f"Açı: {self.yaw:.1f}°  X:{self.robot_x:.2f}  Y:{self.robot_y:.2f}")

        if self.hedef_ok_ciziliyor and self.mouse_start_pos and self.mouse_current_pos:
            ressam.setPen(QPen(QColor(0, 255, 0, 200), 3))
            
            x1 = self.mouse_start_pos.x()
            y1 = self.mouse_start_pos.y()
            x2 = self.mouse_current_pos.x()
            y2 = self.mouse_current_pos.y()
            
            ressam.drawLine(x1, y1, x2, y2)
            
            angle = math.atan2(y2 - y1, x2 - x1)
            arrow_size = 15
            
            ressam.setBrush(QColor(0, 255, 0, 200))
            arrow_head = QPolygonF()
            arrow_head.append(QPointF(x2, y2))
            arrow_head.append(QPointF(x2 - arrow_size * math.cos(angle - math.pi / 6),
                                      y2 - arrow_size * math.sin(angle - math.pi / 6)))
            arrow_head.append(QPointF(x2 - arrow_size * math.cos(angle + math.pi / 6),
                                      y2 - arrow_size * math.sin(angle + math.pi / 6)))
            ressam.drawPolygon(arrow_head)

        ressam.setPen(QPen(QColor("#ffffff"), 1))
        ressam.setFont(QFont("Arial", 10, QFont.Bold))
        ressam.drawText(20, 30, "TAKTİK LİDAR VE NAVİGASYON")
        ressam.drawText(20, 50, f"Menzil: {self.maksimum_menzil}m | Tespit: {len(self.noktalar)} Engel")

        # GNSS (kullanicinin istegi: bu ekranda da GPS konumu gorunsun)
        ressam.setFont(QFont("Arial", 9, QFont.Bold))
        if self.gnss_enlem is not None and self.gnss_boylam is not None:
            ressam.setPen(QPen(QColor("#00d4ff"), 1))
            ressam.drawText(20, 68, f"GNSS: {self.gnss_enlem:.6f}, {self.gnss_boylam:.6f}")
        else:
            ressam.setPen(QPen(QColor("#556677"), 1))
            ressam.drawText(20, 68, "GNSS: sinyal yok")

        # GPS KATKISI (2026-09-05, kullanıcı isteği: "gpsin konum
        # tahminine olan katkısı da yazılsın") - konum_birlestirici.py'nin
        # (araç) GPS düzeltmesi hakkında yayınladığı canlı durum metni.
        # Renk: aktif düzeltme = camgöbeği, bekleniyor/durdu = soluk sarı.
        if self.gps_katki_metni:
            if "katkısı:" in self.gps_katki_metni.lower():
                ressam.setPen(QPen(QColor("#3ddc84"), 1))
            else:
                ressam.setPen(QPen(QColor("#ccaa33"), 1))
            ressam.drawText(20, 86, self.gps_katki_metni)
        else:
            ressam.setPen(QPen(QColor("#556677"), 1))
            ressam.drawText(20, 86, "GPS katkısı: araçtan veri bekleniyor")

        ressam.setFont(QFont("Arial", 11, QFont.Bold))
        if self.pitch > 3.0:
            ressam.setPen(QPen(QColor("#00ff00"), 1))
            ressam.drawText(int(merkez_x + 35), int(merkez_y - 10), f"▲ BURUN YUKARI (+{self.pitch:.1f}°)")
        elif self.pitch < -3.0:
            ressam.setPen(QPen(QColor("#ff6600"), 1))
            ressam.drawText(int(merkez_x + 35), int(merkez_y - 10), f"▼ BURUN AŞAĞI ({self.pitch:.1f}°)")

        if abs(self.roll) > 3.0:
            yon = "SOLA" if self.roll < 0 else "SAĞA"
            ressam.setPen(QPen(QColor("#00bfff"), 1))
            ressam.drawText(int(merkez_x + 35), int(merkez_y + 10), f"📐 {yon} YATIK ({abs(self.roll):.1f}°)")

        # İMLEÇ KOORDİNAT HUD'U (bkz. __init__/mouseMoveEvent notu) -
        # tıklamadan ÖNCE tam olarak hangi /odom koordinatına basılacağını
        # gösterir, hassas nokta atmayı kolaylaştırır.
        if self._imlec_dunya_x is not None:
            ressam.setPen(QPen(QColor("#0b0e14"), 3))
            ressam.setFont(QFont("Arial", 11, QFont.Bold))
            metin = f"İmleç → X:{self._imlec_dunya_x:.2f}  Y:{self._imlec_dunya_y:.2f}"
            ressam.drawText(20, y - 14, metin)
            ressam.setPen(QPen(QColor("#00ff88"), 1))
            ressam.drawText(20, y - 14, metin)
        _bolum['arac+metin'] = (time.perf_counter() - _b) * 1000


# ==============================================================================
# 4. ANA YÖNETİCİ KÖPRÜ (Orijinal QThread Motorunu Çalıştırır)
# ==============================================================================
class HaritaYoneticisi:
    def __init__(self, layout):
        self.offset_roll = 0.0
        self.offset_pitch = 0.0
        self.offset_yaw = 0.0
        # True baslatiliyor: ilk IMU okumasi geldiginde otomatik olarak sifir
        # kabul edilsin (kullanicinin acilista SIFIRLA'ya basmasina gerek kalmaz).
        # "SIFIRLA" butonu istendiginde bunu tekrar True yapmaya devam ediyor.
        self.sifirla_istendi = True

        self.gcs_govde = QWidget()
        self.gcs_govde.setStyleSheet("""
            QWidget { background-color: #07090e; color: #ffffff; }
            QLabel { color: #e0e6ed; font-family: 'Arial'; font-weight: bold; }
            QPushButton { 
                background-color: #16202c; border: 2px solid #0088cc; 
                border-radius: 8px; color: #ffffff; font-weight: bold; padding: 10px;
                font-size: 13px;
            }
            QPushButton:hover { background-color: #1e2e40; border-color: #00ffcc; color: #00ffcc; }
            QPushButton:pressed { background-color: #0088cc; color: #000000; }
        """)
        
        ana_layout = QHBoxLayout(self.gcs_govde)
        ana_layout.setContentsMargins(5, 5, 5, 5)
        ana_layout.setSpacing(15)

        sol_panel = QFrame()
        sol_panel.setFixedWidth(360)
        sol_panel.setStyleSheet("background-color: #0d1117; border: 1px solid #1e2836; border-radius: 15px;")
        sol_layout = QVBoxLayout(sol_panel)
        sol_layout.setContentsMargins(15, 15, 15, 15)

        baslik = QLabel("TUFAN TELEMETRİ PANELS")
        baslik.setStyleSheet("font-size: 15px; color: #00ffcc; border-bottom: 2px solid #1e2836; padding-bottom: 10px;")
        sol_layout.addWidget(baslik)

        self.suni_ufuk = SuniUfukEkrani()
        sol_layout.addWidget(self.suni_ufuk)

        dijital_ekran = QFrame()
        dijital_ekran.setStyleSheet("background-color: #11161f; border-radius: 10px; padding: 10px;")
        dijital_layout = QVBoxLayout(dijital_ekran)

        self.lbl_roll = QLabel("Roll  (Yatma)    : 0.0°")
        self.lbl_pitch = QLabel("Pitch (Yunuslama): 0.0°")
        self.lbl_yaw = QLabel("Yaw   (Pusula)   : 0.0°")
        self.lbl_lidar = QLabel("Lidar Engeller : Bekleniyor...")
        
        for lbl in [self.lbl_roll, self.lbl_pitch, self.lbl_yaw, self.lbl_lidar]:
            lbl.setStyleSheet("font-size: 13px; padding: 4px; color: #00ffcc;")
            dijital_layout.addWidget(lbl)

        sol_layout.addWidget(dijital_ekran)

        self.btn_sifirla = QPushButton("SENSÖR KONUMUNU SIFIRLA (TARE)")
        self.btn_sifirla.clicked.connect(self.konum_sifirla)
        sol_layout.addWidget(self.btn_sifirla)

        self.lbl_durum = QLabel("ROS 2 Bağlantısı Bekleniyor...")
        self.lbl_durum.setStyleSheet("font-size: 11px; color: #ffaa00; padding-top: 5px;")
        sol_layout.addWidget(self.lbl_durum)

        sol_layout.addStretch()
        ana_layout.addWidget(sol_panel)

        self.taktik_radar = TaktikRadarEkrani()

        self.uydu_harita = UyduHaritaWidget()

        self.harita_sekmeleri = QTabWidget()
        self.harita_sekmeleri.setStyleSheet(
            "QTabWidget::pane { border: none; } "
            "QTabBar::tab { background: #11161f; color: #e0e6ed; padding: 8px 18px; "
            "font-weight: bold; } "
            "QTabBar::tab:selected { background: #0088cc; color: #000000; }"
        )
        self.harita_sekmeleri.addTab(self.taktik_radar, "TAKTİK LİDAR")
        self.harita_sekmeleri.addTab(self.uydu_harita, "UYDU HARİTASI")
        ana_layout.addWidget(self.harita_sekmeleri)

        layout.addWidget(self.gcs_govde)

        self.ros_motoru = Ros2GcsMotoru()
        self.ros_motoru.lidar_sinyal.connect(self.lidar_guncelle)
        self.ros_motoru.imu_sinyal.connect(self.imu_guncelle)
        self.ros_motoru.baglanti_sinyali.connect(lambda msg: self.lbl_durum.setText(msg))

        self.ros_motoru.global_plan_sinyal.connect(self.taktik_radar.guncelle_global_plan)
        self.ros_motoru.local_plan_sinyal.connect(self.taktik_radar.guncelle_local_plan)
        self.ros_motoru.costmap_sinyal.connect(self.taktik_radar.guncelle_costmap)
        self.ros_motoru.gps_katki_sinyali.connect(self.taktik_radar.gps_katki_guncelle)
        self.ros_motoru.konum_sinyal.connect(self.taktik_radar.guncelle_konum)
        self.ros_motoru.gnss_sinyal.connect(self._gnss_guncelle)
        self.ros_motoru.rtk_durum_sinyali.connect(self.uydu_harita.rtk_durum_guncelle)

        # ÇOKLU NOKTA ROTASI (2026-09-01, kullanıcı isteği): "ROTAYI GÖNDER"
        # tıklanınca TaktikRadarEkrani'nin biriktirdiği liste
        # Ros2GcsMotoru.coklu_hedef_gonder'e iletilir; ilerleme/sonuç metni
        # de geri panelde gösterilmek üzere bağlanır.
        self.taktik_radar.rota_gonder_istendi.connect(self.ros_motoru.coklu_hedef_gonder)
        self.ros_motoru.rota_durum_sinyali.connect(self.taktik_radar.rota_durum_guncelle)

        self.ros_motoru.start()

    def _gnss_guncelle(self, enlem, boylam):
        self.uydu_harita.konum_guncelle(enlem, boylam, self.taktik_radar.yaw)
        # Kullanicinin istegi: GNSS konumu taktik lidar ekraninda da gorunsun.
        self.taktik_radar.guncelle_gnss(enlem, boylam)

    def hedefi_ilet(self, x, y, q_z, q_w):
        self.ros_motoru.hedefe_git(x, y, q_z, q_w)

    def silah_kamera_guncelle(self, pixmap):
        """main.py::video_ekrana_bas'tan çağrılır - SADECE Navigasyon sayfası
        görünürken. Kullanıcı isteği (2026-09-02): ayrı bir kutu DEĞİL,
        Suni Ufuk'un gökyüzü/yer dolgusunun YERİNE geçer (bkz.
        SuniUfukEkrani.silah_kare_guncelle/paintEvent)."""
        self.suni_ufuk.silah_kare_guncelle(pixmap)

    def konum_sifirla(self):
        self.sifirla_istendi = True
        self.lbl_durum.setText("Sensör Konumu Sıfırlandı (0°, 0°, 0°)")
        self.lbl_durum.setStyleSheet("font-size: 11px; color: #00ff00; padding-top: 5px;")

    def lidar_guncelle(self, noktalar):
        # GEÇİCİ PROFİLLEME (2026-09-06) - bkz. scan_callback'teki not:
        # emit ile bu slot'un GERÇEKTEN çalıştığı an arasındaki fark =
        # sinyalin Qt kuyruğunda bekleme süresi. Sürekli artıyorsa kuyruk
        # birikiyordur (kullanıcının "sonra kötüleşiyor" tarifi).
        _emit_t = getattr(self.ros_motoru, '_son_lidar_emit_t', None)
        if _emit_t is not None:
            _bekleme_ms = (time.perf_counter() - _emit_t) * 1000.0
            _gecmis = getattr(self, '_kuyruk_gecmis', None)
            if _gecmis is None:
                _gecmis = self._kuyruk_gecmis = []
            _gecmis.append(_bekleme_ms)
            if len(_gecmis) >= 30:
                self._kuyruk_gecmis = []
                print(f"[KUYRUK-GECİKME] lidar sinyali GUI'ye ulaşma: "
                      f"ort={sum(_gecmis)/len(_gecmis):.1f}ms "
                      f"en_kötü={max(_gecmis):.1f}ms (0'a yakın olmalı; "
                      f"sürekli artıyorsa kuyruk birikiyor)")
        self.lbl_lidar.setText(f"Lidar Engeller : {len(noktalar)} Nokta Alındı")
        self.taktik_radar.guncelle_veri(noktalar, self.taktik_radar.roll,
                                        self.taktik_radar.pitch, self.taktik_radar.yaw)
        # Kullanicinin istegi: LiDAR nokta bulutu uydu haritasi uzerinde de
        # (gercek konuma/yone gore bindirilmis) gorunsun.
        self.uydu_harita.lidar_guncelle(noktalar)

    def imu_guncelle(self, roll, pitch, yaw):
        if self.sifirla_istendi:
            self.offset_roll = roll
            self.offset_pitch = pitch
            self.offset_yaw = yaw
            self.sifirla_istendi = False

        net_roll = roll - self.offset_roll
        net_pitch = pitch - self.offset_pitch
        net_yaw = yaw - self.offset_yaw

        if net_pitch > 3.0:
            pitch_yazi = "BURUN YUKARI"
        elif net_pitch < -3.0:
            pitch_yazi = "BURUN AŞAĞI"
        else:
            pitch_yazi = "DÜZ"

        if net_roll > 3.0:
            roll_yazi = "SAĞA YATIK"
        elif net_roll < -3.0:
            roll_yazi = "SOLA YATIK"
        else:
            roll_yazi = "DÜZ"

        self.lbl_roll.setText(f"Roll  (Yatma)    : {net_roll:6.1f}°  [{roll_yazi}]")
        self.lbl_pitch.setText(f"Pitch (Yunuslama): {net_pitch:6.1f}°  [{pitch_yazi}]")
        self.lbl_yaw.setText(f"Yaw   (Pusula)   : {net_yaw:.1f}°")

        self.suni_ufuk.guncelle_veri(net_roll, net_pitch)
        # ONEMLI: burada net_yaw (kullanicinin "SIFIRLA" ile ayarladigi
        # kozmetik referans) DEGIL, HAM/duzeltilmis mutlak yaw gonderiliyor.
        # TaktikRadarEkrani.yaw; costmap donusu, hedef tiklama donusumu ve
        # arac ikonu donusu icin kullaniliyor - bunlarin hepsi /odom (Nav2)
        # ile AYNI mutlak referansi paylasmali, yoksa "SIFIRLA"ya basmak
        # hedef yerlesimini kaydirir (navigasyon matematigi kullanicinin
        # kozmetik gösterge sifirlamasindan etkilenmemeli).
        self.taktik_radar.guncelle_veri(self.taktik_radar.noktalar, net_roll, net_pitch, yaw)
        # Kullanicinin istegi: IMU (roll/pitch) uydu haritasi uzerinde de gorunsun.
        self.uydu_harita.imu_guncelle(net_roll, net_pitch)

    def kapat(self):
        if hasattr(self, 'ros_motoru') and self.ros_motoru:
            self.ros_motoru.stop()