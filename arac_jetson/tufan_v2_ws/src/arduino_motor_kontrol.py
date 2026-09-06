#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Bool, String
from sensor_msgs.msg import Imu
import serial
import serial.tools.list_ports
import time

# Arduino Uno'nun resmi USB kimlikleri (VID:PID). Bu kart tipi bazen USB
# tanimlayicisinda "Uno" metnini ayri vermiyor (bkz. arduino_uno_portu_bul),
# bu yuzden VID:PID eslesmesi asil guvenilir yontem.
ARDUINO_UNO_VID_PID = {
    (0x2341, 0x0043),  # Uno R3
    (0x2341, 0x0001),  # Uno R3 (eski bootloader)
    (0x2A03, 0x0043),  # Uno R3 (bazi resmi lisansli klonlar)
}

class ArduinoMotorKontrol(Node):
    def __init__(self):
        super().__init__('arduino_motor_kontrol_uydusu')

        # --- SERİ PORT (USB) BAĞLANTISI ---
        # Sabit /dev/ttyACMx yerine, sistemdeki USB seri cihazlari tarayip
        # "Arduino Uno" olarak taniyan porta baglaniyoruz (Cube Orange gibi
        # diger ACM cihazlari farkli numaralara kaysa bile dogru portu bulur).
        self.seri_port = None
        self._son_port_arama_zamani = 0.0
        self.baudrate = 115200
        self.arduino = None
        self.baglanti_kur()

        # --- PPM AYARLARI ---
        self.ppm_merkez = 1500  # Durma sinyali
        self.ppm_min = 1000     # Tam geri sinyali
        self.ppm_max = 2000     # Tam ileri sinyali

        # --- ABONELİK: Merkezi Yöneticiden Gelen Palet Hızları [-255.0, +255.0] ---
        # BEST_EFFORT: surekli tekrar yayinlanan kontrol sinyali icin
        # RELIABLE QoS'un onay/yeniden gonderim yukunu kaldiriyoruz -
        # periyodik birkac saniyelik tikanmalara sebep oluyordu (canli
        # olculdu, arayuz tarafinda ayni degisiklik yapildi).
        palet_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Float32MultiArray,
            '/palet_hizlari',
            self.palet_callback,
            palet_qos
        )
        self.get_logger().info("🔌 ARDUINO PPM KÖPRÜSÜ AKTİF: [-255, 255] verileri [1000, 2000] mikrosaniyeye dönüştürülüyor...")

        # --- YENİ: ACİL DURDURMA KİLİDİ (2026-08-31) ---
        # Arayüzdeki ACİL DURDUR butonu zaten /arac_komut'a "EMERGENCY_STOP_CMD"
        # yayinliyordu AMA bu dugum o topic'i HIC DINLEMIYORDU - PWM sadece BIR
        # KEZ sifirlaniyor, hemen sonraki /palet_hizlari mesaji (joystick/otonom)
        # üzerine yazabiliyordu. Artik GERCEK bir KILIT var: EMERGENCY_STOP_CMD
        # gelince palet_callback TAMAMEN devre disi kalir (gelen HICBIR PWM
        # islenmez) - "DEVAM_CMD" gelene kadar. Kullanicinin kendi tarifiyle:
        # "komutlari keserek araci durduracak" - tek seferlik sifirlama degil.
        self._kilitli = False
        self.create_subscription(String, '/arac_komut', self._komut_callback, 10)

        # --- YENİ: Arayüzdeki "MOTORLAR" göstergesi için gerçek bağlantı durumu ---
        # Sadece bu node'un durumunu bildirir (var olan hiçbir davranışı değiştirmez).
        self._baglanti_durum_pub = self.create_publisher(Bool, '/arduino_baglanti_durumu', 10)
        self.create_timer(0.5, self._baglanti_durumu_yayinla)

        # --- YENİ: DÜZ GİDİŞTE OTOMATİK YÖN DÜZELTMESİ - PI KONTROLCÜ (2026-08-31) ---
        # Kullanici istegi: manuel surusteki (klavye VEYA joystick, ikisi de
        # /palet_hizlari'a ayni sekilde yaziyor) ileri/geri komutlarinda
        # IMU'yu dinleyip sapma varsa duzelt. ONCE CANLI VERI DOGRULAMASI
        # YAPILDI: /imu/data 99Hz'de gercek/canli, durgunken yaw 0.0000 derece
        # std sapmayla kararli. AMA ayni testte kalibrasyon durumu
        # {sys:0, gyro:3, accel:0, mag:0} bulundu - konum_birlestirici.py'nin
        # kendi dosya-basi notundaki bilinen bulguyla BIREBIR ayni durum:
        # ivmeolcer/manyetometre kalibre degilken, motor akimi degisince
        # BNO055'in FUZE EDILMIS (mutlak) yaw'i 2.14 derece SICRAMISTI -
        # yani PID'nin surekli mutlak/fuze yaw'a guvenmesi TEHLIKELI olurdu
        # (motor akimini degistirdiginde olusan sahte bir "sapma"yi
        # duzeltmeye calisip kendi kendini besleyebilir). Bu yuzden BUNUN
        # YERINE SADECE GYRO (acisal hiz, angular_velocity.z) kullaniliyor -
        # bu eksen zaten tam kalibre (gyro:3) VE manyetik girisimden
        # ETKILENMIYOR (konum_birlestirici.py dosya basi notu). Referans,
        # HER "duz gidis" segmentinin BASLANGICINDA sifirlanan bir gyro
        # INTEGRALI (sadece o an suren duz surus boyunca biriken sapma,
        # omur boyu/mutlak bir aci DEGIL) - boylece manyetometre kalibrasyon
        # sorunundan tamamen bagimsiz VE sinirsiz surunme riski yok.
        #
        # AKTIVASYON: ham_sol_pwm ve ham_sag_pwm AYNI ISARETTE (ikisi de
        # ileri VEYA ikisi de geri) VE ikisi de sifirdan farkliysa "duz
        # gidis niyeti" var sayilir - PID devrede. Isaretler farkliysa
        # (tank donusu) veya biri sifirsa (pivot) operatorun KASITLI bir
        # donus yaptigi varsayilir, PID devre disi kalir ve bir sonraki duz
        # segment icin entegral sifirlanir. NOT: sol/sag PWM DEGERLERI FARKLI
        # OLABILIR (operator Q/Z/E/C ile elle trim yapmis olabilir) - bu
        # PID'yi engellemez, sadece ISARETLERI ayni olmali.
        #
        # YON: standart diferansiyel-suruş kinematigi omega = k*(v_sag-v_sol)
        # - SOLA DON (sol=-,sag=+) -> omega>0 (sol/CCW donus, REP-103'te
        # +yaw=sol) ile TUTARLI, SAGA DON (sol=+,sag=-) -> omega<0 ile
        # TUTARLI (surus_joystick_sistemi.py/telemetri_sistemi.py'deki halen
        # DOGRULANMIS tanimlardan turetildi). Bu formul v_sol/v_sag isaretinde
        # DOGRUSAL oldugu icin ayni duzeltme (sol'a +trim, sag'a -trim)
        # ILERI'de de GERI'de de doğru yon etkisini verir - ayri bir
        # ileri/geri isaret carpani GEREKMEZ.
        self._pid_aktif = True  # varsayilan ACIK (kullanici tercihi) - /yon_pid_aktif ile kapatilabilir
        self._pid_yon_carpani = 1.0  # CANLI TESTTE TERS CIKARSA -1.0 YAP (tek satirlik anahtar)
        self._pid_Kp = 8.0     # PWM birimi / (rad/s) - anlik donus hizina tepki
        self._pid_Ki = 40.0    # PWM birimi / rad - bu segmentte biriken sapmaya tepki
        self._pid_MAX_TRIM = 45.0  # PWM biriminde ust sinir (~255'in %18'i) - PID surucuyu asla ezemez
        self._pid_duz_gidis_aktif = False
        self._pid_entegral_rad = 0.0
        self._pid_son_gyro_z = 0.0
        self._pid_son_imu_zaman = None
        self.create_subscription(Bool, '/yon_pid_aktif', self._pid_aktif_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, qos_profile_sensor_data)

        # --- YENİ: FARLAR (2026-09-01, kullanıcı isteği) ---
        # /farlar (Bool) -> Arduino Uno'nun 2. ve 3. pinine bağlı röle (AKTİF-
        # DÜŞÜK: pin 0'a çekilince farlar AÇILIR). AYNI seri portu (motor
        # Arduino'su) kullanıyor - ayrı bir node/port AÇILMIYOR (kullanıcının
        # açık isteği: "iki kod aynı seri portu açmasın"). Protokol
        # "SOL_PPM,SAG_PPM\n"'den "SOL_PPM,SAG_PPM,FAR\n"'e genişletildi
        # (bkz. .ino dosyasindaki 2026-09-01 notu - eski 2-alanli format da
        # hala calisir, 3. alan opsiyonel). Son gonderilen PPM degerleri
        # saklanir ki far durumu DEGISTIGINDE (palet_hizlari beklemeden)
        # ANINDA, hareketi ETKILEMEDEN iletilebilsin - motor komutu her
        # zaman GUNCEL PPM'i tekrar yazar (dur komutu GONDERMEZ).
        self._far_durumu = False
        self._son_sol_ppm = self.ppm_merkez
        self._son_sag_ppm = self.ppm_merkez
        self.create_subscription(Bool, '/farlar', self._far_cb, 10)

        # FREN (2026-09-06, kullanici istegi): yer istasyonundaki FIZIKSEL
        # buton (eskiden manuel/otonom gecisi) artik freni ac/kapa yapiyor.
        # Buraya SADECE HEDEF DURUM (True=acik, False=kapali) gelir; motoru
        # 3 saniye dondurme isini ARDUINO kendi yapar (bkz. sketch'teki
        # kenar-tetiklemeli, bloklamayan blok). Sureyi Jetson'da saymak
        # YANLIS olurdu: baglanti o 3 saniye icinde koparsa fren yarim
        # kalirdi; Arduino'da sayilinca hareket her halukarda tamamlanir.
        self._fren_durumu = False
        self.create_subscription(Bool, '/fren_komut', self._fren_cb, 10)

    def _fren_cb(self, msg):
        yeni = bool(msg.data)
        if yeni == self._fren_durumu:
            return  # ayni durum tekrar gelirse Arduino zaten tetiklenmez
        self._fren_durumu = yeni
        self.get_logger().warn(
            "\U0001F17F\uFE0F FREN: " + ("ACILIYOR" if yeni else "KAPANIYOR")
            + " (motor 3 sn calisacak)")
        if not self.arduino or not self.arduino.is_open:
            return
        # NOT: acil durdurma KILIDI freni ENGELLEMEZ - kilitliyken de fren
        # kumanda edilebilmeli (arac durdurulduktan sonra fren cekmek
        # gerekebilir). Kilit sadece PALET komutlarini kesiyor.
        try:
            self.arduino.write(self._seri_komut().encode('utf-8'))
        except Exception as e:
            self.get_logger().error("\u26A0\uFE0F Fren komutu gonderme hatasi: " + str(e))

    def _seri_komut(self):
        """Arduino paketi: "SOL_PPM,SAG_PPM,FAR,FREN\n" - tek yerden
        uretilir ki alan eklendiginde bir cagri yeri unutulmasin.

        *** KILITLIYKEN NOTR (2026-09-06, kod incelemesinde bulundu) ***
        Acil durdurma kilidi aktifken palet_callback zaten hic yazmiyor,
        AMA _fren_cb KASITLI OLARAK kilitten muaf (arac durdurulduktan
        sonra fren cekilebilmeli). O yazma, paketin ilk iki alanina
        _son_sol_ppm/_son_sag_ppm'i (acil stop ANINDAKI son surus hizi)
        koyuyordu - yani fren dugmesine basmak kilitli araci SON HIZIYLA
        yeniden hareket ettirebilirdi. Kilitliyken PPM alanlari her zaman
        merkez (dur) degerine zorlanir; far/fren alanlari calismaya devam
        eder."""
        if self._kilitli:
            sol = sag = self.ppm_merkez
        else:
            sol, sag = self._son_sol_ppm, self._son_sag_ppm
        return "%d,%d,%d,%d\n" % (sol, sag,
                                   int(self._far_durumu), int(self._fren_durumu))

    def _far_cb(self, msg):
        self._far_durumu = bool(msg.data)
        self.get_logger().info(f"💡 Farlar: {'AÇIK' if self._far_durumu else 'KAPALI'}")
        if self._kilitli or not self.arduino or not self.arduino.is_open:
            return
        try:
            self.arduino.write(self._seri_komut().encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"⚠️ Far komutu gönderme hatası: {e}")

    def _pid_aktif_cb(self, msg):
        self._pid_aktif = bool(msg.data)
        if not self._pid_aktif:
            self._pid_duz_gidis_aktif = False
            self._pid_entegral_rad = 0.0
        self.get_logger().info(f"🧭 Yön düzeltme PID: {'AÇIK' if self._pid_aktif else 'KAPALI'}")

    def _imu_cb(self, msg):
        gyro_z = msg.angular_velocity.z
        self._pid_son_gyro_z = gyro_z
        simdi = time.monotonic()
        if self._pid_duz_gidis_aktif and self._pid_son_imu_zaman is not None:
            dt = simdi - self._pid_son_imu_zaman
            # Ani/anormal dt (ör. IMU yayini kesintiye ugramis) entegrali bozmasin
            if 0.0 < dt < 0.5:
                self._pid_entegral_rad += gyro_z * dt
        self._pid_son_imu_zaman = simdi

    def _pid_trim_hesapla(self, ham_sol_pwm, ham_sag_pwm):
        """Duz gidis niyeti varsa gyro-tabanli PI duzeltme trim'i doner,
        yoksa 0.0 doner (ve durumu/entegrali sifirlar)."""
        if not self._pid_aktif:
            return 0.0

        duz_gidis = (
            ham_sol_pwm != 0.0 and ham_sag_pwm != 0.0
            and (ham_sol_pwm > 0) == (ham_sag_pwm > 0)
        )

        if not duz_gidis:
            if self._pid_duz_gidis_aktif:
                self.get_logger().info("🧭 Yön düzeltme: dönüş/dur algılandı, PID sıfırlandı.")
            self._pid_duz_gidis_aktif = False
            self._pid_entegral_rad = 0.0
            return 0.0

        if not self._pid_duz_gidis_aktif:
            # YENİ duz gidis segmenti basliyor - referans/entegral sifirlanir
            self._pid_duz_gidis_aktif = True
            self._pid_entegral_rad = 0.0

        trim = self._pid_yon_carpani * (
            self._pid_Kp * self._pid_son_gyro_z + self._pid_Ki * self._pid_entegral_rad
        )
        return max(min(trim, self._pid_MAX_TRIM), -self._pid_MAX_TRIM)

    def _baglanti_durumu_yayinla(self):
        msg = Bool()
        msg.data = bool(self.arduino and self.arduino.is_open)
        self._baglanti_durum_pub.publish(msg)

    def arduino_uno_portu_bul(self):
        """
        Takili USB-seri cihazlari tarar. Once bilinen Arduino Uno VID:PID
        kombinasyonuna, sonra USB tanimlayicisinda "arduino uno" gecen porta,
        bulamazsa yedek olarak sadece "arduino" gecen ilk porta baglanir.
        Hicbiri yoksa None doner.
        """
        yedek = None
        for p in serial.tools.list_ports.comports():
            if (p.vid, p.pid) in ARDUINO_UNO_VID_PID:
                return p.device
            etiket = " ".join(filter(None, [p.manufacturer, p.description, p.product])).lower()
            if "arduino uno" in etiket:
                return p.device
            if yedek is None and "arduino" in etiket:
                yedek = p.device
        return yedek

    def baglanti_kur(self):
        # Ardarda gelen basarisiz deneme cagrilarinin USB'yi surekli
        # taramasini onlemek icin kisa bir bekleme uyguluyoruz.
        simdi = time.monotonic()
        if simdi - self._son_port_arama_zamani < 2.0:
            return
        self._son_port_arama_zamani = simdi

        port = self.arduino_uno_portu_bul()
        if port is None:
            self.get_logger().error("❌ ARDUINO UNO BULUNAMADI: Takili USB portlarinda 'Arduino Uno' gorunmuyor!")
            self.get_logger().warn("⚠️ Lütfen USB kablosunu kontrol edin!")
            return

        self.seri_port = port
        try:
            self.arduino = serial.Serial(self.seri_port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Arduino'nun resetlenip kendine gelmesi için bekleme
            self.get_logger().info(f"✅ Arduino Uno ile seri iletişim kuruldu: {self.seri_port}")
        except Exception as e:
            self.get_logger().error(f"❌ ARDUINO BAĞLANTI HATASI ({self.seri_port}): {e}")
            self.get_logger().warn("⚠️ Lütfen USB kablosunu ve port adını kontrol edin!")

    def pwm_to_ppm(self, pwm_degeri, ters_yon=False):
        """
        ROS 2'den gelen [-255, 255] aralığındaki hızı,
        ESC/Servo'nun anladığı [1000, 2000] mikrosaniye aralığına doğrusal oranlar.
        """
        # Sınırla
        pwm_sinirli = max(min(float(pwm_degeri), 255.0), -255.0)
        
        if ters_yon:
            pwm_sinirli = -pwm_sinirli

        # -255 -> -500ms ekle | 0 -> 0ms ekle | +255 -> +500ms ekle
        eklenecek_ms = int((pwm_sinirli / 255.0) * 500.0)
        ppm = self.ppm_merkez + eklenecek_ms
        
        # Son güvenlik mandalı
        return max(min(ppm, self.ppm_max), self.ppm_min)

    def _komut_callback(self, msg):
        komut = msg.data.strip()
        if komut == "EMERGENCY_STOP_CMD":
            self._kilitli = True
            self.get_logger().warn("🛑 ACİL DURDURMA KİLİTLENDİ - /palet_hizlari komutları 'DEVAM_CMD' gelene kadar YOK SAYILACAK")
            if self.arduino and self.arduino.is_open:
                try:
                    dur_komutu = f"{self.ppm_merkez},{self.ppm_merkez}\n"
                    self.arduino.write(dur_komutu.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"⚠️ Acil durdurma sirasinda seri port yazma hatasi: {e}")
        elif komut == "DEVAM_CMD":
            self._kilitli = False
            self.get_logger().info("✅ Acil durdurma kilidi AÇILDI - normal /palet_hizlari işlenmeye devam ediyor")

    def palet_callback(self, msg):
        if self._kilitli:
            # KİLİTLİYKEN gelen HİÇBİR PWM komutu işlenmez - joystick/klavye/
            # otonom fark etmeksizin. Sadece "DEVAM_CMD" bunu açabilir.
            return
        if not self.arduino or not self.arduino.is_open:
            self.baglanti_kur()
            return

        try:
            ham_sol_pwm = msg.data[0]
            ham_sag_pwm = msg.data[1]

            # YÖN DÜZELTME PID (bkz. __init__ yorumu): duz gidiste gyro
            # tabanli trim - sol'a +trim, sag'a -trim (trim>0 = sola kayma
            # duzeltmesi, yukaridaki "sag>sol -> sola kivrilir" notuyla
            # TUTARLI: sag'i azaltip sol'u artirmak sola kivrilmayi azaltir).
            trim = self._pid_trim_hesapla(ham_sol_pwm, ham_sag_pwm)
            if trim != 0.0:
                ham_sol_pwm = max(min(ham_sol_pwm + trim, 255.0), -255.0)
                ham_sag_pwm = max(min(ham_sag_pwm - trim, 255.0), -255.0)

            # DÜZELTME: Eski kodunuzda W'ye basıldığında Sol Motor (1500 - Hız), Sağ Motor (1500 + Hız) yapılıyordu.
            # Yani motorlardan birinin fiziksel montaj yönü ters!
            # Bu yüzden sol motora 'ters_yon=True' parametresi vererek bu simetriyi mekanik olarak sağlıyoruz.
            sol_ppm = self.pwm_to_ppm(ham_sol_pwm, ters_yon=True)
            sag_ppm = self.pwm_to_ppm(ham_sag_pwm, ters_yon=False)
            self._son_sol_ppm = sol_ppm
            self._son_sag_ppm = sag_ppm

            # Arduino paket formatı: "SOL_PPM,SAG_PPM,FAR,FREN\n"
            # Örn: "1400,1650,0,1\n" (Sol hafif ileri, Sağ orta-üst ileri,
            # farlar kapalı, fren açık)
            komut = self._seri_komut()

            self.arduino.write(komut.encode('utf-8'))

            # TEŞHİS LOG (gecici): gercekten seri porta yazilan komutu goster.
            self.get_logger().info(f"Gönderilen PPM -> Sol: {sol_ppm} us | Sağ: {sag_ppm} us | ham: {komut.strip()}")
            
        except Exception as e:
            self.get_logger().error(f"⚠️ Seri port veri gönderme hatası: {e}")

    def stop(self):
        if self.arduino and self.arduino.is_open:
            # Güvenlik için kapanmadan önce 1500 (Tam Dur / Merkez) sinyali bas
            dur_komutu = f"{self.ppm_merkez},{self.ppm_merkez}\n"
            self.arduino.write(dur_komutu.encode('utf-8'))
            self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    dugum = ArduinoMotorKontrol()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.stop()
        dugum.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
