#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool
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

        # --- YENİ: Arayüzdeki "MOTORLAR" göstergesi için gerçek bağlantı durumu ---
        # Sadece bu node'un durumunu bildirir (var olan hiçbir davranışı değiştirmez).
        self._baglanti_durum_pub = self.create_publisher(Bool, '/arduino_baglanti_durumu', 10)
        self.create_timer(0.5, self._baglanti_durumu_yayinla)

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

    def palet_callback(self, msg):
        if not self.arduino or not self.arduino.is_open:
            self.baglanti_kur()
            return

        try:
            ham_sol_pwm = msg.data[0]
            ham_sag_pwm = msg.data[1]

            # DÜZELTME: Eski kodunuzda W'ye basıldığında Sol Motor (1500 - Hız), Sağ Motor (1500 + Hız) yapılıyordu.
            # Yani motorlardan birinin fiziksel montaj yönü ters!
            # Bu yüzden sol motora 'ters_yon=True' parametresi vererek bu simetriyi mekanik olarak sağlıyoruz.
            sol_ppm = self.pwm_to_ppm(ham_sol_pwm, ters_yon=True)
            sag_ppm = self.pwm_to_ppm(ham_sag_pwm, ters_yon=False)

            # Arduino'nun okuyabileceği paket formatı: "SOL_PPM,SAG_PPM\n"
            # Örn: "1400,1650\n" (Sol hafif ileri, Sağ orta-üst ileri -> Araç yumuşakça sola kıvrılır!)
            komut = f"{sol_ppm},{sag_ppm}\n"
            
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
