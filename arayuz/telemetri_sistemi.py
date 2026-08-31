import time
import subprocess
import threading
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from std_msgs.msg import String, Float32, Bool, Int32, Float32MultiArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu
    from geometry_msgs.msg import Twist
except Exception as e:
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    QoSHistoryPolicy = None
    String = None
    Float32 = None
    Bool = None
    Int32 = None
    Float32MultiArray = None
    Odometry = None
    LaserScan = None
    Imu = None
    Twist = None
    ROS2_IMPORT_ERROR = str(e)
else:
    ROS2_IMPORT_ERROR = None

try:
    from mavros_msgs.msg import GPSRAW
except Exception:
    # konum_birlestirici.py'deki AYNI durum: mavros_msgs kurulu degilse
    # (ör. sadece ~/ros2_humble sourcelandi) GNSS gostergesi sessizce PASİF
    # kalir, geri kalan telemetri etkilenmez.
    GPSRAW = None

from PyQt5.QtCore import QThread, pyqtSignal
from terminal_widget import VARSAYILAN_HOST as WIFI_YEDEK_IP

# Yarista aracla arayuz arasindaki kablosuz linki Ubiquiti nokta-nokta
# kopru sagliyor (WiFi degil). Radyonun kendi yonetim IP'si (192.168.1.21)
# ICMP ping'i engelliyor (canli test edildi) - bu yuzden dogrudan ARAC
# Jetson'a ping atip UCTAN UCA baglanti kalitesini olcuyoruz; operatoru
# asil ilgilendiren de zaten "araca gercekten ulasabiliyor muyum" sorusu.
# IP degisirse burasi guncellenmeli.
UBIQUITI_LINK_HEDEF_IP = "192.168.1.22"
# YENİ: Ubiquiti tamamen kopukken (%0) gösterge sessizce "bağlantı yok"
# göstermesin - araç WiFi üzerinden (terminal_widget.py'nin kullandığı AYNI
# IP, bkz. import) hala erişilebilir olabilir (nitekim terminal zaten bu IP
# ile SSH kuruyor). Ubiquiti kopukken WiFi'ye otomatik düşülür, gösterge
# hangi linkin gerçekte kullanıldığını da bildirir (bkz. main.py wifi_arayuz_guncelle).

class TelemetriThread(QThread):
    # --- PyQt5 SİNYALLERİ ---
    hiz_sinyali = pyqtSignal(str)
    batarya_sinyali = pyqtSignal(str)
    etap_sinyali = pyqtSignal(str)
    yer_batarya_sinyali = pyqtSignal(str)
    
    guc_durum_sinyali = pyqtSignal(bool)
    gps_durum_sinyali = pyqtSignal(bool)
    kamera_durum_sinyali = pyqtSignal(bool)
    lidar_durum_sinyali = pyqtSignal(bool)
    imu_durum_sinyali = pyqtSignal(bool)
    # (otonom_hazir, manuel_hazir) -- aractaki Jetson'da otonom yigin
    # (goal_manager) ve/veya manuel surus koprusu (arduino) calisiyor mu.
    mod_durum_sinyali = pyqtSignal(bool, bool)
    wifi_durum_sinyali = pyqtSignal(int, str)  # (yuzde, kaynak: "UBIQUITI" veya "WIFI")
    # Silah/turret TF02-Pro lidar'inin hedefe olan mesafesi (metre).
    hedef_mesafe_sinyali = pyqtSignal(float)
    
    log_sinyali = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.surus_node = None
        self.cmd_pub = None
        self.mod_pub = None
        self.palet_pub = None
        self.ros2_bagli = False

        if rclpy is None or Node is None or String is None or Float32 is None or Bool is None or Int32 is None or Float32MultiArray is None:
            self.log_sinyali.emit("⚠️ ROS2 bulunamadığı için telemetri yayıncısı devre dışı bırakıldı.")
            return

        if not rclpy.ok():
            rclpy.init()

        # --- GUVENLIK KRITIK: surus komutlari (cmd/mod/palet) icin AYRI,
        # MINIMAL bir node + kendi izole thread'i/executor'u. Asagidaki
        # self.node'da (9 telemetri aboneligiyle) AYNI node'da tutulunca,
        # DDS kesif/eslesme mekanizmasi periyodik olarak (~3sn'de bir,
        # gercek donanimla canli olculdu) tikaniyor ve /palet_hizlari
        # yayini kesintiye ugruyordu - "geliyor gidiyor" sikayeti buradan
        # kaynaklaniyordu (WiFi/ag DEGIL - izole testlerle elendi). 300kg'lik
        # aracin sürüş komutu ASLA baska hicbir seye bagimli/gecikmeli
        # olmamali, bu yuzden kendi kucucuk node'unda tamamen izole.
        self.surus_node = Node('tufan_yer_istasyonu_surus')
        self.cmd_pub = self.surus_node.create_publisher(String, '/arac_komut', 10)
        self.mod_pub = self.surus_node.create_publisher(String, '/surus_modu', 10)
        # BEST_EFFORT: /palet_hizlari surekli (10Hz) tekrar yayinlanan bir
        # kontrol sinyali - kaybolan tek bir ornegin onemi yok, 0.1sn sonra
        # yenisi geliyor zaten. Varsayilan RELIABLE QoS'un yeniden
        # gonderim/onay (ack/retransmit) mekanizmasi, bu aglar uzerinde
        # periyodik birkac saniyelik tikanmalara sebep oluyordu (canli
        # olculdu). BEST_EFFORT bu overhead'i tamamen ortadan kaldirir.
        palet_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.palet_pub = self.surus_node.create_publisher(Float32MultiArray, '/palet_hizlari', palet_qos)
        self.surus_node.create_timer(0.1, self.surekli_yayin_dongusu)

        # --- GECICI: yarismadan once araç ve silah icin AYRI joystick
        # olacak; simdilik TEK joystick, "Joystick Hedefi" ile arac/silah
        # arasinda elle secilerek paylasiliyor. Ayni sürus-kritik/dusuk
        # gecikme geregi oldugundan ayni izole surus_node'da yayinlaniyor. ---
        self.joystick_hedefi = "ARAC"  # "ARAC" veya "SILAH"
        self.silah_modu_pub = self.surus_node.create_publisher(String, '/silah_modu', 10)
        self.turret_cmd_pub = self.surus_node.create_publisher(Twist, '/turret_manuel_cmd', 10)
        self.silah_ates_pub = self.surus_node.create_publisher(Bool, '/silah_ates_manuel', 10)

        # --- YENİ: Düz gidişte otomatik yön düzeltme (araç tarafı gyro-PID)
        # AÇ/KAPA anahtarı - araçtaki arduino_motor_kontrol.py varsayılan
        # AÇIK başlıyor, kullanıcı sahada beklenmedik davranış görürse tek
        # tıkla kapatabilsin diye (bkz. main.py _yon_pid_butonu_ekle). ---
        self.yon_pid_pub = self.surus_node.create_publisher(Bool, '/yon_pid_aktif', 10)

        # --- Genel telemetri/gosterge dugumu (SADECE abonelikler - gecikmesi
        # sürüşü ASLA etkilemez, ayri node'da oldugu icin) ---
        self.node = Node('tufan_yer_istasyonu')

        # NOT: /arac_hiz, /arac_batarya, /lidar_durum topic'lerinin aractaki
        # gercek karsiligi yok (canli dogrulandi) -- hicbir zaman veri gelmiyordu,
        # panel hep varsayilan/son deger gosteriyordu. Hiz artik gercek /odom'dan
        # (konum_birlestirici.py'nin yayinladigi, twist.linear.x) okunuyor.
        self.node.create_subscription(Float32, '/arac_hiz', self.hiz_cb, 10)
        self.node.create_subscription(Int32, '/arac_batarya', self.batarya_cb, 10)
        self.node.create_subscription(Bool, '/lidar_durum', self.lidar_cb, 10)
        if Odometry is not None:
            self.node.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        if LaserScan is not None:
            self.node.create_subscription(LaserScan, '/scan', self.scan_heartbeat_cb, 10)
        if Bool is not None:
            self.node.create_subscription(Bool, '/arduino_baglanti_durumu', self.motor_durum_cb, 10)
        if Imu is not None:
            self.node.create_subscription(Imu, '/imu/data', self.imu_heartbeat_cb, 10)
        if Int32 is not None:
            self.node.create_subscription(Int32, '/goal_manager_heartbeat', self.otonom_heartbeat_cb, 10)
        if Int32 is not None:
            # ETAP göstergesi (dashboard'da label_etapNo) - Designer'da
            # sabit "8" yazıyordu, etap_sinyali TANIMLIYDI ama hiç emit
            # edilmiyordu (gps_durum_sinyali ile AYNI durum). Artık
            # tabela_etap_yoneticisi.py'nin (araç) yayınladığı gerçek
            # tespit edilen etap numarasına bağlı.
            self.node.create_subscription(Int32, '/guncel_etap', self.guncel_etap_cb, 10)
        if GPSRAW is not None:
            # GNSS gercekten aktif mi - konum_birlestirici.py'de zaten
            # dogrulanmis AYNI kaynak (fix_type: 0=yok,1=fixsiz,2=2D,3=3D,
            # 4=DGPS,5=RTK Float,6=RTK Fixed). "ETKİN" esigi olarak 3D fix
            # (>=3) kullaniliyor - konum_birlestirici.py'nin kendi
            # gps_heading_min_fix_type varsayilaniyla (3) TUTARLI, RTK'siz de
            # (sadece 3D fix) GNSS'in temelde calistigini gostermek icin
            # yeterli bir esik (RTK ozel durumu ayri, NTRIP/RTK panelinde).
            self.node.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.gps_raw_cb, 10)
        self.node.create_subscription(Float32, '/turret_hedef_mesafe', self.hedef_mesafe_cb, 10)
        self.son_lidar_zamani = 0.0
        self.anlik_lidar_durumu = None
        self.son_imu_zamani = 0.0
        self.anlik_imu_durumu = None
        self.son_otonom_zamani = 0.0
        self.son_gps_zamani = 0.0
        self.son_gps_fix_type = 0
        self.anlik_gps_durumu = None
        self.manuel_hazir = False
        self.anlik_mod_durumu = None
        self.ros2_bagli = True
        
        self.arac_modu = "MANUEL"
        self.surus_kaynagi = "KLAVYE"  # "KLAVYE" veya "JOYSTICK" - ayni anda tek kaynak yazsin diye
        self.anlik_sol_pwm = 0.0
        self.anlik_sag_pwm = 0.0

        # PWM araligi: aracin motorlari 85'in altinda fiziksel olarak
        # donmuyor (gercek donanimda dogrulandi), bu yuzden ALT SINIR sabit.
        # UST SINIR ayarlar ekranindaki kutudan (kolay surus icin) canli
        # degistirilebilir, varsayilan 255 (tam guc).
        self.pwm_alt_sinir = 85.0
        self.pwm_ust_sinir = 255.0

        self.sol_hiz_kademesi = 85.0
        self.sag_hiz_kademesi = 85.0
        
        self.son_komut_zamani = time.time()
        self.guvenlik_zaman_asimi = 0.8  # Süre uzatıldı. Basılı tutarken kesilmeleri önler. 
        
        # --- YENİ EKLENEN: SİSTEM MERKEZİ WATCHDOG TAKİBİ ---
        self.son_telemetri_zamani = 0.0  # İlk açılışta çevrimdışı (PASİF) başlasın
        # ----------------------------------------------------
        
        # UBIQUITI LINK KALITESI: ROS'tan gelen bir sey degil, arac Jetson'a
        # periyodik ping atarak olculuyor (bkz. _wifi_kontrol). ONEMLI: bu
        # ROS2 timer'i (create_timer) DEGIL, ayri bir thread - ping
        # bloklayici olabilir, ROS2 executor'inin İÇİNDE calistirilirsa
        # /palet_hizlari dahil TUM yayin/abonelik durabilir (daha once
        # ayni sebeple nmcli icin canli olcumle dogrulanmis bir sorundu).
        self._wifi_thread_calisiyor = True
        threading.Thread(target=self._wifi_kontrol_dongusu, daemon=True).start()

        # Surus spin thread'i EN SONDA baslatiliyor - yukaridaki TUM durum
        # degiskenleri (arac_modu, surus_kaynagi, anlik_sol_pwm vb.) hazir
        # olmadan surekli_yayin_dongusu (0.1sn'de bir tetiklenir) calisirsa
        # AttributeError ile cokuyordu (yaris durumu, canli tespit edildi).
        self._surus_thread_calisiyor = True
        threading.Thread(target=self._surus_spin_dongusu, daemon=True).start()

        self.log_yaz("✅ Telemetri ve Bağımsız Palet Sistemi (Fiziksel Eşleşmeli) Başlatıldı.")

    def hiz_cb(self, msg):
        self.son_telemetri_zamani = time.time() # Veri geldi, kalp atışı güncellendi!
        self.hiz_sinyali.emit(f"{round(msg.data, 1)} m/s")

    def odom_cb(self, msg):
        self.son_telemetri_zamani = time.time()  # Veri geldi, kalp atışı güncellendi!
        hiz = abs(msg.twist.twist.linear.x)
        self.hiz_sinyali.emit(f"{round(hiz, 1)} m/s")

    def batarya_cb(self, msg): 
        self.son_telemetri_zamani = time.time() # Veri geldi, kalp atışı güncellendi!
        self.batarya_sinyali.emit(f"%{msg.data}")

    def lidar_cb(self, msg):
        self.son_telemetri_zamani = time.time() # Veri geldi, kalp atışı güncellendi!
        self.lidar_durum_sinyali.emit(msg.data)

    def motor_durum_cb(self, msg):
        # Gercek kaynak: arduino_motor_kontrol.py /arduino_baglanti_durumu
        # yayinliyor (self.arduino.is_open). Bu artik dogrudan bir panel
        # etiketine degil, "MANUEL HAZIR" durumuna besleniyor (bkz.
        # surekli_yayin_dongusu -> mod_durum_sinyali).
        self.son_telemetri_zamani = time.time()
        self.manuel_hazir = bool(msg.data)

    def scan_heartbeat_cb(self, msg):
        # /lidar_durum topic'i gercekte hic yayinlanmiyor -- lidar durumunu
        # gercek /scan mesajlarinin gelme sikligindan (heartbeat) cikariyoruz.
        self.son_telemetri_zamani = time.time()
        self.son_lidar_zamani = time.time()

    def imu_heartbeat_cb(self, msg):
        # IMU (bno055 /imu/data) gercekten veri gonderiyor mu -- heartbeat.
        self.son_telemetri_zamani = time.time()
        self.son_imu_zamani = time.time()

    def otonom_heartbeat_cb(self, msg):
        # goal_manager_node'un heartbeat'i -- otonom (Nav2/goal_manager)
        # yiginin calisip calismadiginin gercek isareti.
        self.son_telemetri_zamani = time.time()
        self.son_otonom_zamani = time.time()

    def guncel_etap_cb(self, msg):
        self.son_telemetri_zamani = time.time()
        self.etap_sinyali.emit(str(msg.data))

    def gps_raw_cb(self, msg):
        # /mavros/gpsstatus/gps1/raw (GPSRAW) -- gercek GNSS fix kalitesi.
        # konum_birlestirici.py'de zaten canli dogrulanmis AYNI alan.
        self.son_telemetri_zamani = time.time()
        self.son_gps_zamani = time.time()
        self.son_gps_fix_type = msg.fix_type

    def hedef_mesafe_cb(self, msg):
        self.hedef_mesafe_sinyali.emit(float(msg.data))

    def _surus_spin_dongusu(self):
        # Surus (cmd/mod/palet) node'unun KENDI izole executor'i - genel
        # telemetri node'undan (9 abonelik) tamamen ayri thread. Bkz.
        # __init__ icindeki guvenlik notu.
        executor = SingleThreadedExecutor()
        executor.add_node(self.surus_node)
        try:
            while rclpy.ok() and self._surus_thread_calisiyor:
                executor.spin_once(timeout_sec=0.02)
        finally:
            executor.remove_node(self.surus_node)

    def _wifi_kontrol_dongusu(self):
        # ROS2 executor'indan tamamen bagimsiz ayri thread (bkz. __init__
        # icindeki not) - ping yavas/bloklayici olabilir, burada
        # bekletmenin ROS2 yayinina hicbir etkisi yok.
        while self._wifi_thread_calisiyor:
            self._wifi_kontrol()
            time.sleep(3.0)

    def _ubiquiti_fiziksel_bagli_mi(self):
        # KRİTİK BULGU (2026-08-31): eskiden sadece UBIQUITI_LINK_HEDEF_IP'ye
        # ping atılıp başarılı olursa "UBIQUITI" deniyordu - ama bu IP'ye
        # giden rota Ubiquiti FİZİKSEL OLARAK BAĞLI DEĞİLKEN BİLE WiFi'nin
        # kendi ağ geçidi üzerinden gidebiliyor (`ip route get` ile canlı
        # doğrulandı: rota wlP1p1s0/WiFi üzerinden çıkıyor, Ubiquiti'ye özel
        # ayrı bir arayüz/alt ağ YOK) - yani ping bazen başarılı dönüp
        # yanlışlıkla "UBIQUITI: %40/%100" gösterebiliyordu, Ubiquiti kablosu
        # hiç takılı değilken bile (canlı bildirildi). Gerçek kaynak: Ubiquiti
        # radyosunun bağlı olduğu kablolu Ethernet portunun (enP8p1s0) FİZİKSEL
        # link durumu (carrier) - kablo/radyo yoksa "0" yazar, IP/ping'e hiç
        # bakmadan kesin olarak bilinir. Port adı değişirse burası güncellenmeli.
        try:
            with open('/sys/class/net/enP8p1s0/carrier') as f:
                return f.read().strip() == '1'
        except Exception:
            return False  # arayuz bulunamadi/okunamadi - guvenli varsayim: bagli degil

    def _wifi_kontrol(self):
        # Yarışta gerçek WiFi degil, Ubiquiti nokta-nokta kablosuz köprü
        # kullanılıyor - gösterge artık o linkin kalitesini gösteriyor.
        # 5 hizli ping gonderip basari oranini "%baglanti kalitesi" olarak
        # yayinliyoruz (RSSI yerine gercek paket kaybi - PtP link icin
        # daha anlamli). UBIQUITI_LINK_HEDEF_IP degisirse burasi guncellenmeli.
        if self._ubiquiti_fiziksel_bagli_mi():
            try:
                cikti = subprocess.run(
                    ["ping", "-c", "5", "-i", "0.2", "-W", "1", UBIQUITI_LINK_HEDEF_IP],
                    capture_output=True, text=True, timeout=4.0
                ).stdout
                basarili = cikti.count(" bytes from ")
                yuzde = int(round((basarili / 5.0) * 100))
                if yuzde > 0:
                    self.wifi_durum_sinyali.emit(yuzde, "UBIQUITI")
                    return
            except Exception:
                pass

        # Ubiquiti fiziksel olarak bağlı değil (veya bağlı ama ping alamıyor) -
        # WiFi uzerinden hala erisilebilir
        # olabilir mi diye ayni sekilde dene (terminal_widget.py'nin SSH icin
        # kullandigi ayni IP - araç zaten o agda).
        try:
            cikti = subprocess.run(
                ["ping", "-c", "5", "-i", "0.2", "-W", "1", WIFI_YEDEK_IP],
                capture_output=True, text=True, timeout=4.0
            ).stdout
            basarili = cikti.count(" bytes from ")
            yuzde = int(round((basarili / 5.0) * 100))
            if yuzde > 0:
                self.wifi_durum_sinyali.emit(yuzde, "WIFI")
            else:
                # Ikisi de tamamen kopuk - "UBIQUITI" olarak %0 goster
                # (WIFI:%0 gostermek, denenen-ama-basarisiz WiFi'yi asil
                # baglanti gibi gostermis olurdu - yaniltici).
                self.wifi_durum_sinyali.emit(0, "UBIQUITI")
        except Exception:
            self.wifi_durum_sinyali.emit(0, "UBIQUITI")

    def log_yaz(self, mesaj):
        print(mesaj)
        self.log_sinyali.emit(mesaj)

    def mod_degistir(self, yeni_mod):
        self.arac_modu = yeni_mod
        if self.mod_pub is None:
            self.log_yaz("⚠️ ROS2 yayıncısı yok, sürüş modu değiştirilemedi.")
            return
        msg = String()
        msg.data = yeni_mod
        self.mod_pub.publish(msg)
        self.log_yaz(f"🔄 Sürüş Modu Değiştirildi: {yeni_mod}")
        
        if yeni_mod != "MANUEL":
            self.anlik_sol_pwm = 0.0
            self.anlik_sag_pwm = 0.0
            self.anlik_palet_yayinla()

    def surekli_yayin_dongusu(self):
        if self.mod_pub is None:
            return
        mod_msg = String()
        mod_msg.data = self.arac_modu
        self.mod_pub.publish(mod_msg)

        # --- OTONOM/MANUEL MOD DURUMU ---
        # Otonom: goal_manager_node heartbeat'i geliyor mu (Nav2/goal_manager
        # yigini calisiyor mu). Manuel: arduino_motor_kontrol.py'nin seri
        # baglantisi acik mi (motor_durum_cb ile guncelleniyor).
        otonom_hazir = (time.time() - self.son_otonom_zamani) < 3.0
        mod_durumu = (otonom_hazir, self.manuel_hazir)
        if mod_durumu != self.anlik_mod_durumu:
            self.anlik_mod_durumu = mod_durumu
            self.mod_durum_sinyali.emit(otonom_hazir, self.manuel_hazir)
        # ----------------------------------------------

        # --- LIDAR CANLILIK KONTROLU (gercek /scan heartbeat'i) ---
        lidar_canli_mi = (time.time() - self.son_lidar_zamani) < 1.0
        if lidar_canli_mi != self.anlik_lidar_durumu:
            self.anlik_lidar_durumu = lidar_canli_mi
            self.lidar_durum_sinyali.emit(lidar_canli_mi)
        # ------------------------------------------------------------

        # --- IMU CANLILIK KONTROLU (gercek /imu/data heartbeat'i) ---
        imu_canli_mi = (time.time() - self.son_imu_zamani) < 1.0
        if imu_canli_mi != self.anlik_imu_durumu:
            self.anlik_imu_durumu = imu_canli_mi
            self.imu_durum_sinyali.emit(imu_canli_mi)
        # ------------------------------------------------------------

        # --- GNSS CANLILIK + FIX KALITESI KONTROLU (gercek GPSRAW) ---
        # Hem veri GERCEKTEN akiyor mu (heartbeat, 3sn - gps_bringup.launch.py
        # calismiyorsa mavros GPSRAW hic yayinlamaz) HEM DE fix_type yeterli
        # mi (>=3, 3D fix - konum_birlestirici.py'nin esigiyle tutarli).
        gps_canli_mi = (time.time() - self.son_gps_zamani) < 3.0
        gps_etkin = gps_canli_mi and self.son_gps_fix_type >= 3
        if gps_etkin != self.anlik_gps_durumu:
            self.anlik_gps_durumu = gps_etkin
            self.gps_durum_sinyali.emit(gps_etkin)
        # ------------------------------------------------------------

        if self.arac_modu == "MANUEL":
            if (self.anlik_sol_pwm != 0.0 or self.anlik_sag_pwm != 0.0):
                if (time.time() - self.son_komut_zamani) > self.guvenlik_zaman_asimi:
                    self.anlik_sol_pwm = 0.0
                    self.anlik_sag_pwm = 0.0
                    self.log_yaz("🛑 GÜVENLİK KILIDI: Tuş sinyali kesildi, paletler frenlendi!")

            self.anlik_palet_yayinla()

    def anlik_palet_yayinla(self):
        if self.palet_pub is None:
            return
        palet_msg = Float32MultiArray()
        # DÜZELTME: Arduino tarafı ile uyumlu hale getirildi (0: Sol, 1: Sağ)
        # Eksi işaretleri sisteminizin fiziksel yönüne göre korundu.
        palet_msg.data = [-float(self.anlik_sol_pwm), -float(self.anlik_sag_pwm)]
        self.palet_pub.publish(palet_msg)

    def pwm_ust_sinirini_ayarla(self, deger):
        # Ayarlar ekranindaki (eski "Telefon") kutudan gelir. Alt sinir
        # (85 - motorlarin fiziksel olarak donmeye basladigi deger) SABIT;
        # sadece ust sinir (kolay surus icin tavan hiz) degistirilebilir.
        deger = max(self.pwm_alt_sinir, min(255.0, float(deger)))
        self.pwm_ust_sinir = deger
        # Klavye kademeleri yeni tavani asmasin.
        self.sol_hiz_kademesi = min(self.sol_hiz_kademesi, deger)
        self.sag_hiz_kademesi = min(self.sag_hiz_kademesi, deger)
        self.log_yaz(f"🎚️ PWM üst sınırı {deger:.0f} olarak ayarlandı.")
        return deger

    def _pwm_olcekle(self, oran):
        # -1..1 araligindaki oranli girdiyi (orn. joystick eksen orani)
        # [pwm_alt_sinir, pwm_ust_sinir] araligina olceklendirir. Boylece
        # kucuk bir joystick hareketi bile motoru fiilen donduren minimum
        # PWM'i (85) verir, tam hareket ise guncel tavani (varsayilan 255).
        if abs(oran) < 1e-6:
            return 0.0
        isaret = 1.0 if oran > 0 else -1.0
        buyukluk = min(abs(oran), 1.0)
        return isaret * (self.pwm_alt_sinir + buyukluk * (self.pwm_ust_sinir - self.pwm_alt_sinir))

    def joystick_pwm_gonder(self, sol_oran, sag_oran):
        # surus_joystick_sistemi.py'den HAM ORAN (-1..1) gelir. Yayin ve
        # guvenlik watchdog'u surekli_yayin_dongusu'nda zaten var - burada
        # anlik durumu, guncel PWM sinirlarina olceklenmis olarak, ve
        # watchdog zaman damgasini guncelliyoruz.
        if self.arac_modu != "MANUEL" or self.surus_kaynagi != "JOYSTICK":
            return
        # GECICI paylasimli joystick: hedef SILAH iken araca HICBIR komut
        # gitmesin (fiziksel joystick su an silah nisanlamak icin kullaniliyor).
        if self.joystick_hedefi != "ARAC":
            return
        self.son_komut_zamani = time.time()
        self.anlik_sol_pwm = self._pwm_olcekle(float(sol_oran))
        self.anlik_sag_pwm = self._pwm_olcekle(float(sag_oran))

    def joystick_hedefini_ayarla(self, hedef):
        # hedef: "ARAC" veya "SILAH". Degisirken araci guvenli tarafta
        # birak: SILAH'a gecerken araç PWM'i hemen sifirla (joystick artik
        # ona komut gondermeyecek, eski deger PALET'te takili kalmasin).
        if hedef == self.joystick_hedefi:
            return
        self.joystick_hedefi = hedef
        if hedef == "SILAH":
            self.anlik_sol_pwm = 0.0
            self.anlik_sag_pwm = 0.0
            if self.silah_modu_pub is not None:
                self.silah_modu_pub.publish(String(data="MANUEL"))

    def joystick_turret_gonder(self, x, y):
        # surus_joystick_sistemi.py'den gelen HAM x/y (tank karisimi
        # ONCESI) - aracin joystick kalibrasyonu (merkez/olu bolge/oto-
        # merkezleme/guvenlik siniri) zaten uygulanmis durumda, silah icin
        # ayrica kalibrasyona gerek yok. turret_node.py bunu Twist.linear.x
        # (pan) / linear.y (tilt) olarak bekliyor (bkz. _manuel_cmd_cb).
        if self.joystick_hedefi != "SILAH" or self.turret_cmd_pub is None:
            return
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        self.turret_cmd_pub.publish(msg)

    def silah_ates_ayarla(self, aktif: bool):
        if self.silah_ates_pub is None:
            return
        self.silah_ates_pub.publish(Bool(data=bool(aktif)))

    def yon_pid_ayarla(self, aktif: bool):
        if self.yon_pid_pub is None:
            return
        self.yon_pid_pub.publish(Bool(data=bool(aktif)))

    def hareket_emri_gonder(self, yon):
        if self.cmd_pub is None:
            self.log_yaz("⚠️ ROS2 yayıncısı yok, hareket emri gönderilemedi.")
            return
        if self.arac_modu != "MANUEL" and yon not in ["DUR (SPACE)", "EMERGENCY_STOP_CMD", "DEVAM_CMD"]:
            return
        if self.surus_kaynagi != "KLAVYE" and yon not in ["DUR (SPACE)", "EMERGENCY_STOP_CMD", "DEVAM_CMD"]:
            return

        self.son_komut_zamani = time.time()
        msg = String()

        # --- 0. ACİL DURDURMA KİLİDİ (araçtaki arduino_motor_kontrol.py'nin
        # /arac_komut üzerinde beklediği TAM OLARAK bu iki metni gönderiyoruz -
        # aşağıdaki genel yön/harf komutlarından farklı olarak burada literal
        # metin lazım, "X\n" gibi tek harfli bir komut DEĞİL). Bu iki komut
        # PWM/palet durumuna dokunmaz - kilit araç tarafında yönetiliyor. ---
        if yon in ("EMERGENCY_STOP_CMD", "DEVAM_CMD"):
            msg.data = yon
            self.cmd_pub.publish(msg)
            if yon == "EMERGENCY_STOP_CMD":
                self.anlik_sol_pwm = 0.0
                self.anlik_sag_pwm = 0.0
                self.log_yaz("🛑 ACİL DURDURMA KİLİDİ GÖNDERİLDİ: araçtaki motor komutları kesildi.")
                self.anlik_palet_yayinla()
            else:
                self.log_yaz("✅ DEVAM KOMUTU GÖNDERİLDİ: araçtaki acil durdurma kilidi açılıyor.")
            return

        # --- 1. YÖN VE HAREKET KOMUTLARI ---
        if "İLERİ" in yon: 
            msg.data = "W\n"
            self.anlik_sol_pwm = self.sol_hiz_kademesi
            self.anlik_sag_pwm = self.sag_hiz_kademesi
            self.log_yaz(f"▲ İLERİ -> Sol Palet: {self.anlik_sol_pwm} PWM | Sağ Palet: {self.anlik_sag_pwm} PWM")
            
        elif "GERİ" in yon: 
            msg.data = "S\n"
            self.anlik_sol_pwm = -self.sol_hiz_kademesi
            self.anlik_sag_pwm = -self.sag_hiz_kademesi
            self.log_yaz(f"▼ GERİ -> Sol Palet: {self.anlik_sol_pwm} PWM | Sağ Palet: {self.anlik_sag_pwm} PWM")
            
        elif "SOLA DÖN" in yon: 
            msg.data = "A\n"
            self.anlik_sol_pwm = -self.sol_hiz_kademesi
            self.anlik_sag_pwm = self.sag_hiz_kademesi
            self.log_yaz(f"◀ SOLA TANK DÖNÜŞÜ -> Sol: {self.anlik_sol_pwm} | Sağ: {self.anlik_sag_pwm}")
            
        elif "SAĞA DÖN" in yon: 
            msg.data = "D\n"
            self.anlik_sol_pwm = self.sol_hiz_kademesi
            self.anlik_sag_pwm = -self.sag_hiz_kademesi
            self.log_yaz(f"▶ SAĞA TANK DÖNÜŞÜ -> Sol: {self.anlik_sol_pwm} | Sağ: {self.anlik_sag_pwm}")
            
        # --- 2. SOL PALET BAĞIMSIZ HIZ AYARI (Yön Korumalı) ---
        elif "SOL_HIZ_ARTIR" in yon or "SOL_ARTIR" in yon: 
            msg.data = "Q\n"
            self.sol_hiz_kademesi = min(self.pwm_ust_sinir, self.sol_hiz_kademesi + 20.0)
            if self.anlik_sol_pwm < 0: self.anlik_sol_pwm = -self.sol_hiz_kademesi
            else: self.anlik_sol_pwm = self.sol_hiz_kademesi
            self.log_yaz(f"⬅️ SOL PALET GÜÇLENDİ -> Sol: {self.sol_hiz_kademesi} PWM | Sağ: {self.sag_hiz_kademesi} PWM")
            
        elif "SOL_HIZ_AZALT" in yon or "SOL_AZALT" in yon: 
            msg.data = "Z\n"
            self.sol_hiz_kademesi = max(self.pwm_alt_sinir, self.sol_hiz_kademesi - 20.0)
            if self.anlik_sol_pwm < 0: self.anlik_sol_pwm = -self.sol_hiz_kademesi
            else: self.anlik_sol_pwm = self.sol_hiz_kademesi  
            self.log_yaz(f"⬅️ SOL PALET YAVAŞLADI -> Sol: {self.sol_hiz_kademesi} PWM | Sağ: {self.sag_hiz_kademesi} PWM")
            
        # --- 3. SAĞ PALET BAĞIMSIZ HIZ AYARI (Yön Korumalı) ---
        elif "SAG_HIZ_ARTIR" in yon or "SAG_ARTIR" in yon: 
            msg.data = "E\n"
            self.sag_hiz_kademesi = min(self.pwm_ust_sinir, self.sag_hiz_kademesi + 20.0)
            if self.anlik_sag_pwm < 0: self.anlik_sag_pwm = -self.sag_hiz_kademesi
            else: self.anlik_sag_pwm = self.sag_hiz_kademesi
            self.log_yaz(f"➡️ SAĞ PALET GÜÇLENDİ -> Sol: {self.sol_hiz_kademesi} PWM | Sağ: {self.sag_hiz_kademesi} PWM")
            
        elif "SAG_HIZ_AZALT" in yon or "SAG_AZALT" in yon: 
            msg.data = "C\n"
            self.sag_hiz_kademesi = max(self.pwm_alt_sinir, self.sag_hiz_kademesi - 20.0)
            if self.anlik_sag_pwm < 0: self.anlik_sag_pwm = -self.sag_hiz_kademesi
            else: self.anlik_sag_pwm = self.sag_hiz_kademesi  
            self.log_yaz(f"➡️ SAĞ PALET YAVAŞLADI -> Sol: {self.sol_hiz_kademesi} PWM | Sağ: {self.sag_hiz_kademesi} PWM")

        # --- 4. HIZLARI EŞİTLEME (RESET - R Tuşu) ---
        elif "ESITLE" in yon:
            msg.data = "R\n"
            ortak_hiz = (self.sol_hiz_kademesi + self.sag_hiz_kademesi) / 2.0
            self.sol_hiz_kademesi = round(ortak_hiz, 1)
            self.sag_hiz_kademesi = round(ortak_hiz, 1)
            if self.anlik_sol_pwm < 0: self.anlik_sol_pwm = -self.sol_hiz_kademesi
            else: self.anlik_sol_pwm = self.sol_hiz_kademesi
            if self.anlik_sag_pwm < 0: self.anlik_sag_pwm = -self.sag_hiz_kademesi
            else: self.anlik_sag_pwm = self.sag_hiz_kademesi
            self.log_yaz(f"🔄 HIZLAR EŞİTLENDİ -> Sol: {self.sol_hiz_kademesi} | Sağ: {self.sag_hiz_kademesi}")
            
        # --- 5. DURDURMA VE ACİL FREN ---
        elif "DUR" in yon or "SPACE" in yon or "EMERGENCY" in yon: 
            msg.data = "X\n"
            self.anlik_sol_pwm = 0.0
            self.anlik_sag_pwm = 0.0
            self.log_yaz("🛑 ACİL FREN: Tüm motorlar durduruldu (0.0 PWM).")
            
        else:
            self.log_yaz(f"⚠️ Tanımlanamayan Komut Alındı: '{yon}'")
            
        self.cmd_pub.publish(msg)
        self.anlik_palet_yayinla()

    def run(self):
        if self.node is None:
            return
        # KENDI OZEL EXECUTOR: rclpy.spin(node) paylasilan GLOBAL executor'i
        # kullanir. harita_sistemi.py (Ros2GcsMotoru) ve main.py (LidarThread)
        # de ayni sekilde kendi izole SingleThreadedExecutor'larini kullaniyor
        # (aksi halde "IndexError: wait set index too big" ile cokme riski
        # vardi). Bu thread de ayni izolasyona sahip degildi - /palet_hizlari
        # yayininin duzensiz/kesintili gelmesine (bazen 3sn'ye varan bosluklar)
        # katkida bulunuyor olabilirdi. Digerleriyle TUTARLI hale getiriyoruz.
        self.calisiyor = True
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            while rclpy.ok() and self.calisiyor:
                executor.spin_once(timeout_sec=0.05)
        finally:
            executor.remove_node(self.node)