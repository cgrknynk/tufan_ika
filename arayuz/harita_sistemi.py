import sys
import math
import time
import threading # YENİ: ROS'u kilitlemeyen Action bağlantısı için
# --- ÇEKİRDEK ROS2 / LİDAR / IMU (rota özelliklerinden BAĞIMSIZ) ---
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import LaserScan, Imu, NavSatFix
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer, TransformListener
except Exception as e:
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None
    LaserScan = None
    Imu = None
    NavSatFix = None
    OccupancyGrid = None
    PoseStamped = None
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

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPoint, QRectF, QPointF
from PyQt5.QtGui import (QPainter, QColor, QPen, QFont, QPolygon, 
                         QLinearGradient, QBrush, QRadialGradient, QPolygonF)
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QFrame, QTabWidget)
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

    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.calisiyor = True
        self.son_imu_gonderim = 0.0
        self.latest_yaw_rad = 0.0
        self.nav_to_pose_client = None
        self.goal_pose_pub = None
        self.tf_buffer = None
        self.tf_listener = None

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
        # Uydu harita ekrani icin gercek GNSS konumu (Here4 RTK -> ublox_gps_node -> /fix)
        if NavSatFix is not None:
            self.gnss_sub = self.node.create_subscription(NavSatFix, '/fix', self.gnss_callback, 10)
        # DÜZELTME: bno055 sürücüsü '/bno055/imu' diye bir topic YAYINLAMIYOR
        # (canlı doğrulandı) - füzyonlu/kalibreli veri gerçekte '/imu/data'da.
        # Eski topic adında hiç publisher olmadığı için IMU hiçbir zaman veri almıyordu.
        self.imu_sub = self.node.create_subscription(Imu, '/imu/data', self.imu_callback, imu_qos)

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

        # Nav2 Abonelikleri (nav2_msgs kurulu değilse bu kısım atlanır, Lidar/IMU etkilenmez)
        if ActionClient and NavigateToPose and Path:
            self.nav_to_pose_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
            self.global_plan_sub = self.node.create_subscription(Path, '/plan', self.global_plan_callback, nav2_qos)
            self.local_plan_sub = self.node.create_subscription(Path, '/local_plan', self.local_plan_callback, nav2_qos)
            self.baglanti_sinyali.emit("🟢 ROS 2 Aktif: Lidar & IMU & Nav2 Rota")
        else:
            print(f"[HARITA] Nav2 mesaj paketleri (nav2_msgs) bulunamadı, rota özelliği devre dışı: {NAV2_IMPORT_ERROR}")
            self.baglanti_sinyali.emit("🟡 ROS 2 Aktif: Lidar & IMU (Nav2 rota paketleri eksik!)")

        print("🚀 ROS 2 Motoru Başlatıldı: QThread Modu.")

        # ORİJİNAL ÇALIŞAN SPIN DÖNGÜSÜ (Kilitlenmez) - kendi izole executor'ımızla
        while rclpy.ok() and self.calisiyor:
            self.executor.spin_once(timeout_sec=0.005)

    def scan_callback(self, msg):
        gecerli_noktalar = []
        for i, mesafe in enumerate(msg.ranges):
            if math.isinf(mesafe) or math.isnan(mesafe):
                continue
            if 0.05 < mesafe < 15.0: 
                aci = msg.angle_min + i * msg.angle_increment + math.pi
                x = mesafe * math.cos(aci)
                y = mesafe * math.sin(aci)
                gecerli_noktalar.append((x, y))
                
        self.lidar_sinyal.emit(gecerli_noktalar)

    def imu_callback(self, msg):
        q = msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return

        su_an = time.time()
        if su_an - self.son_imu_gonderim < 0.016:
            return
        self.son_imu_gonderim = su_an

        # costmap_callback icin: konum_birlestirici.py /odom TF'ini HAM (montaj
        # duzeltmesi UYGULANMAMIS) /imu/data yonelimiyle yayinliyor (canli
        # dogrulandi -> ayni anda duzeltilmis yaw 92 derece iken /odom'un
        # yaw'i 2 derece). costmap bu HAM referansla insa edildigi icin, geri
        # donusturme de AYNI ham yaw'i kullanmali -- duzeltilmis (goruntu icin
        # kullanilan) yaw'i DEGIL.
        self.latest_yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # --- IMU MONTAJ DÜZELTMESİ (fiziksel eksen <-> araç gövde ekseni) ---
        # Bu araca monte edilmiş IMU'nun ham eksenleri REP-103 gövde eksenine
        # (X=ileri, Y=sol, Z=yukarı) uymuyor: canlı doğrulandı -> IMU'nun
        # X ekseni aracın SOLUNA, Y ekseni aracın ARKASINA, Z ekseni YUKARI
        # bakıyor. Bu, Z ekseni etrafında sabit -90°'lik bir montaj farkı demek
        # (gövde_X = -imu_Y, gövde_Y = imu_X, gövde_Z = imu_Z). Roll/Pitch/Yaw'ı
        # ham quaterniondan doğrudan standart formülle çekmek (eskisi gibi)
        # açıları birbirine karıştırıyordu; çünkü Euler açıları basit eksen
        # takasıyla değil, quaternion çarpımıyla düzeltilmesi gerekiyor:
        # q_govde = q_imu ⊗ q_montaj  (q_montaj = Z ekseninde -90° dönüş).
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
        yaw = math.degrees(math.atan2(sin_z, cos_z))
        # NOT: self.latest_yaw_rad BURADA GUNCELLENMIYOR -- costmap icin
        # kullanilan ham yaw yukarida (fonksiyon basinda) zaten ayarlandi.

        self.imu_sinyal.emit(roll, pitch, yaw)

        # Araç resminin üstünde canlı X/Y konumunu göstermek için (eski masaüstü
        # koduna benzer şekilde) o anki odom->base_footprint konumunu da yayınla.
        if self.tf_buffer is not None:
            try:
                tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                self.konum_sinyal.emit(tf.transform.translation.x, tf.transform.translation.y)
            except Exception:
                pass

    def global_plan_callback(self, msg):
        poses = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.global_plan_sinyal.emit(poses)
        
    def local_plan_callback(self, msg):
        poses = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.local_plan_sinyal.emit(poses)

    def gnss_callback(self, msg):
        # status.status < 0 (STATUS_NO_FIX) ise konum gecersiz, gonderme.
        if msg.status.status < 0:
            return
        self.gnss_sinyal.emit(msg.latitude, msg.longitude)

    def costmap_callback(self, msg):
        # local_costmap 'rolling_window: true' ile geliyor (nav2_params.yaml'da
        # dogrulandi) -> arac HER ZAMAN gridin tam merkezinde. Bu sayede TF
        # (odom->base_footprint) lookup'ina hic gerek kalmadan robot konumunu
        # gridin geometrik merkezinden hesaplayabiliyoruz -- eski kod burada
        # tf_buffer kullaniyordu ama tf_buffer/TransformListener kararsizliga
        # (art arda cökme) sebep oldugu icin kalici olarak devre disi (bkz.
        # run() icindeki not); bu yontem o riske hic girmiyor.
        # Yon icin TF yerine self.latest_yaw_rad kullaniliyor -- bu, montaj
        # duzeltmesi UYGULANMAMIS HAM /imu/data yaw'i (bkz. imu_callback basi).
        # ONEMLI: konum_birlestirici.py (aractaki /odom TF yayincisi) da
        # base_footprint donusu icin AYNI ham /imu/data yonelimini dogrudan
        # kullaniyor (canli kaynak kodda dogrulandi) -- yani costmap da bu ham
        # referansla insa ediliyor. Duzeltilmis (goruntu paneli icin kullanilan)
        # yaw kullanilirsa costmap onlarca derece yanlis donuk gorunur.
        res = msg.info.resolution
        genislik0 = msg.info.width
        yukseklik0 = msg.info.height
        if res <= 0.0 or genislik0 == 0 or yukseklik0 == 0:
            return

        robot_x = msg.info.origin.position.x + (genislik0 / 2.0) * res
        robot_y = msg.info.origin.position.y + (yukseklik0 / 2.0) * res
        yaw = self.latest_yaw_rad
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

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
                    dx = world_x - robot_x
                    dy = world_y - robot_y
                    # dünya-çerçevesi farkını araç gövdesine (ileri/sol) çevir
                    ileri = dx * cos_y + dy * sin_y
                    sol = -dx * sin_y + dy * cos_y
                    noktalar.append((ileri, sol))

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
        # (x,y) zaten TaktikRadarEkrani'nde aracın O ANKİ konumuna göre (mouse
        # sürüklemesinden) hesaplanıyor, yani zaten araç-göreceli - bu da
        # 'base_footprint' ile birebir eşleşiyor. frame_id='base_footprint' ile
        # canlı test ettim: hedef kabul ediliyor ve /plan gerçekten doluyor.
        msg = PoseStamped()
        msg.header.frame_id = 'base_footprint'
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

    def guncelle_veri(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        ressam = QPainter(self)
        ressam.setRenderHint(QPainter.Antialiasing)
        
        g = self.width()
        y = self.height()
        merkez_x, merkez_y = g / 2, y / 2
        yaricap = min(g, y) / 2 - 20

        ressam.save()
        ressam.translate(merkez_x, merkez_y)
        ressam.setClipRect(int(-yaricap), int(-yaricap), int(yaricap * 2), int(yaricap * 2))
        
        ressam.rotate(self.roll)
        pitch_piksel = self.pitch * 2.5 

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
        ressam.end()


# ==============================================================================
# 3. SAĞ PANEL: KOORDİNELİ 3D TAKTİK RADAR (Nav2 + Mouse Kontrolü)
# ==============================================================================
class TaktikRadarEkrani(QFrame):
    hedef_belirlendi = pyqtSignal(float, float, float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(450, 450)
        self.setStyleSheet("background-color: #0b0e14; border: 2px solid #1e2836; border-radius: 15px;")
        
        self.noktalar = []
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.maksimum_menzil = 6.0 
        
        self.global_plan = []
        self.local_plan = []
        self.costmap_noktalari = []
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.mouse_start_pos = None
        self.mouse_current_pos = None
        self.hedef_ok_ciziliyor = False
        
        self.setMouseTracking(True) 

    def guncelle_veri(self, noktalar, roll, pitch, yaw):
        self.noktalar = noktalar
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.update()
        
    def guncelle_global_plan(self, poses):
        self.global_plan = poses
        self.update()
        
    def guncelle_local_plan(self, poses):
        self.local_plan = poses
        self.update()

    def guncelle_costmap(self, noktalar):
        self.costmap_noktalari = noktalar
        self.update()

    def guncelle_konum(self, x, y):
        self.robot_x = x
        self.robot_y = y

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_start_pos = event.pos()
            self.mouse_current_pos = event.pos()
            self.hedef_ok_ciziliyor = True
            self.update()

    def mouseMoveEvent(self, event):
        if self.hedef_ok_ciziliyor:
            self.mouse_current_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.hedef_ok_ciziliyor:
            self.hedef_ok_ciziliyor = False
            
            g = self.width()
            y_h = self.height()
            merkez_x, merkez_y = g / 2, y_h / 2
            olcek = min(merkez_x, merkez_y) / self.maksimum_menzil
            
            start_x = self.mouse_start_pos.x()
            start_y = self.mouse_start_pos.y()
            end_x = event.pos().x()
            end_y = event.pos().y()
            
            delta_px = start_x - merkez_x
            delta_py = start_y - merkez_y

            # Ekranda "yukarı" HER ZAMAN sabit (odom) referans yönünü gösterir;
            # rota çizgileri (global/local plan) de bu yüzden yaw'a göre
            # DÖNDÜRÜLMEDEN çiziliyor (bkz. paintEvent - rotate() bloğundan
            # önceler). Fare tıklaması da önce bu sabit çerçevedeki
            # ileri/sol bileşenlerine çevrilir:
            ileri_sabit = -(delta_py / olcek)
            sol_sabit = -(delta_px / olcek)

            dx = end_x - start_x
            dy = end_y - start_y

            hedef_yaw_sabit = math.atan2(-dx, -dy)

            # DÜZELTME: Nav2'ye gönderilen hedef 'base_footprint' (araca göre)
            # çerçevede olmalı - yani aracın O ANKİ yönüne (self.yaw) göre
            # döndürülmüş olması gerekir. Bu adım eksik olduğu için araç düz
            # durmuyorken (self.yaw ≈ 0 değilken) ekranda sola çizilen ok
            # gerçekte gövdeye göre başka bir yöne (ör. aşağı/arkaya) denk
            # geliyordu.
            yaw_rad = math.radians(self.yaw)
            cos_y = math.cos(yaw_rad)
            sin_y = math.sin(yaw_rad)
            hedef_ros_x = ileri_sabit * cos_y + sol_sabit * sin_y
            hedef_ros_y = -ileri_sabit * sin_y + sol_sabit * cos_y
            hedef_yaw = hedef_yaw_sabit - yaw_rad

            q_z = math.sin(hedef_yaw / 2.0)
            q_w = math.cos(hedef_yaw / 2.0)
            
            self.hedef_belirlendi.emit(hedef_ros_x, hedef_ros_y, q_z, q_w)
            self.update()

    def paintEvent(self, event):
        ressam = QPainter(self)
        ressam.setRenderHint(QPainter.Antialiasing)
        
        g = self.width()
        y = self.height()
        merkez_x, merkez_y = g / 2, y / 2
        olcek = min(merkez_x, merkez_y) / self.maksimum_menzil

        grad = QRadialGradient(merkez_x, merkez_y, min(merkez_x, merkez_y))
        grad.setColorAt(0.0, QColor("#0f1c2b"))
        grad.setColorAt(1.0, QColor("#0b0e14"))
        ressam.fillRect(0, 0, g, y, grad)

        ressam.setPen(QPen(QColor("#1a3b5c"), 1, Qt.DashLine))
        for m in range(1, int(self.maksimum_menzil) + 1):
            r = m * olcek
            ressam.drawEllipse(int(merkez_x - r), int(merkez_y - r), int(r * 2), int(r * 2))
            if m % 2 == 0:
                ressam.setFont(QFont("Arial", 8))
                ressam.setPen(QPen(QColor("#0088cc"), 1))
                ressam.drawText(int(merkez_x + 5), int(merkez_y - r + 15), f"{m}m")
                ressam.setPen(QPen(QColor("#1a3b5c"), 1, Qt.DashLine))

        ressam.setPen(QPen(QColor("#1a3b5c"), 1))
        ressam.drawLine(int(merkez_x), 0, int(merkez_x), y)
        ressam.drawLine(0, int(merkez_y), g, int(merkez_y))

        if self.global_plan:
            ressam.setPen(QPen(QColor(0, 150, 255, 200), 3, Qt.DashLine))
            path_polygon = QPolygonF()
            for rx, ry in self.global_plan:
                p_x = merkez_x - (ry * olcek)
                p_y = merkez_y - (rx * olcek)
                path_polygon.append(QPointF(p_x, p_y))
            ressam.drawPolyline(path_polygon)
            
        if self.local_plan:
            ressam.setPen(QPen(QColor(255, 200, 0, 255), 4))
            path_polygon = QPolygonF()
            for rx, ry in self.local_plan:
                p_x = merkez_x - (ry * olcek)
                p_y = merkez_y - (rx * olcek)
                path_polygon.append(QPointF(p_x, p_y))
            ressam.drawPolyline(path_polygon)

        ressam.save()
        ressam.translate(merkez_x, merkez_y)
        ressam.rotate(-self.yaw)

        # Costmap "engel izi" (RViz'deki gibi kalıcı/sönümlü iz) - canlı lidar
        # noktalarının ALTINDA, soluk turuncu olarak çiziliyor.
        if self.costmap_noktalari:
            ressam.setBrush(QColor(255, 140, 0, 90))
            ressam.setPen(Qt.NoPen)
            for x_m, y_m in self.costmap_noktalari:
                p_x = int(-y_m * olcek)
                p_y = int(-x_m * olcek)
                ressam.drawRect(p_x - 2, p_y - 2, 4, 4)

        ressam.setBrush(QColor("#ff0033"))
        ressam.setPen(Qt.NoPen)
        for x_m, y_m in self.noktalar:
            p_x = int(-y_m * olcek)
            p_y = int(-x_m * olcek)
            ressam.drawEllipse(p_x - 3, p_y - 3, 6, 6)

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

        ressam.end()


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
        self.taktik_radar.hedef_belirlendi.connect(self.hedefi_ilet)

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
        self.ros_motoru.konum_sinyal.connect(self.taktik_radar.guncelle_konum)
        self.ros_motoru.gnss_sinyal.connect(self._gnss_guncelle)

        self.ros_motoru.start()

    def _gnss_guncelle(self, enlem, boylam):
        self.uydu_harita.konum_guncelle(enlem, boylam, self.taktik_radar.yaw)

    def hedefi_ilet(self, x, y, q_z, q_w):
        self.ros_motoru.hedefe_git(x, y, q_z, q_w)

    def konum_sifirla(self):
        self.sifirla_istendi = True
        self.lbl_durum.setText("Sensör Konumu Sıfırlandı (0°, 0°, 0°)")
        self.lbl_durum.setStyleSheet("font-size: 11px; color: #00ff00; padding-top: 5px;")

    def lidar_guncelle(self, noktalar):
        self.lbl_lidar.setText(f"Lidar Engeller : {len(noktalar)} Nokta Alındı")
        self.taktik_radar.guncelle_veri(noktalar, self.taktik_radar.roll, 
                                        self.taktik_radar.pitch, self.taktik_radar.yaw)

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
        self.taktik_radar.guncelle_veri(self.taktik_radar.noktalar, net_roll, net_pitch, net_yaw)

    def kapat(self):
        if hasattr(self, 'ros_motoru') and self.ros_motoru:
            self.ros_motoru.stop()