import time
import subprocess
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32, Bool, Int32, Float32MultiArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu
except Exception as e:
    rclpy = None
    Node = None
    String = None
    Float32 = None
    Bool = None
    Int32 = None
    Float32MultiArray = None
    Odometry = None
    LaserScan = None
    Imu = None
    ROS2_IMPORT_ERROR = str(e)
else:
    ROS2_IMPORT_ERROR = None
from PyQt5.QtCore import QThread, pyqtSignal

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
    wifi_durum_sinyali = pyqtSignal(int)
    # Silah/turret TF02-Pro lidar'inin hedefe olan mesafesi (metre).
    hedef_mesafe_sinyali = pyqtSignal(float)
    
    log_sinyali = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.cmd_pub = None
        self.mod_pub = None
        self.palet_pub = None
        self.ros2_bagli = False

        if rclpy is None or Node is None or String is None or Float32 is None or Bool is None or Int32 is None or Float32MultiArray is None:
            self.log_sinyali.emit("⚠️ ROS2 bulunamadığı için telemetri yayıncısı devre dışı bırakıldı.")
            return

        if not rclpy.ok():
            rclpy.init()
        self.node = Node('tufan_yer_istasyonu')
        
        self.cmd_pub = self.node.create_publisher(String, '/arac_komut', 10)
        self.mod_pub = self.node.create_publisher(String, '/surus_modu', 10)
        self.palet_pub = self.node.create_publisher(Float32MultiArray, '/palet_hizlari', 10)
        
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
        self.node.create_subscription(Float32, '/turret_hedef_mesafe', self.hedef_mesafe_cb, 10)
        self.son_lidar_zamani = 0.0
        self.anlik_lidar_durumu = None
        self.son_imu_zamani = 0.0
        self.anlik_imu_durumu = None
        self.son_otonom_zamani = 0.0
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
        
        self.node.create_timer(0.1, self.surekli_yayin_dongusu)
        # WIFI: bu makinenin (arayuz jetsonu) kendi gercek sinyal gucu --
        # ROS'tan gelen bir sey degil, yerel NetworkManager'dan okunuyor.
        self.node.create_timer(3.0, self._wifi_kontrol)
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

    def hedef_mesafe_cb(self, msg):
        self.hedef_mesafe_sinyali.emit(float(msg.data))

    def _wifi_kontrol(self):
        try:
            cikti = subprocess.run(
                ["nmcli", "-t", "-f", "active,signal", "dev", "wifi"],
                capture_output=True, text=True, timeout=2.0
            ).stdout
            for satir in cikti.splitlines():
                aktif, _, sinyal = satir.partition(":")
                if aktif == "yes" and sinyal.strip().isdigit():
                    self.wifi_durum_sinyali.emit(int(sinyal))
                    return
            self.wifi_durum_sinyali.emit(0)  # aktif wifi baglantisi yok
        except Exception:
            pass  # nmcli yoksa/basarisiz olursa panel son bilinen degerde kalir

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
        self.son_komut_zamani = time.time()
        self.anlik_sol_pwm = self._pwm_olcekle(float(sol_oran))
        self.anlik_sag_pwm = self._pwm_olcekle(float(sag_oran))

    def hareket_emri_gonder(self, yon):
        if self.cmd_pub is None:
            self.log_yaz("⚠️ ROS2 yayıncısı yok, hareket emri gönderilemedi.")
            return
        if self.arac_modu != "MANUEL" and yon not in ["DUR (SPACE)", "EMERGENCY_STOP_CMD"]:
            return
        if self.surus_kaynagi != "KLAVYE" and yon not in ["DUR (SPACE)", "EMERGENCY_STOP_CMD"]:
            return

        self.son_komut_zamani = time.time()
        msg = String()
        
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
        rclpy.spin(self.node)