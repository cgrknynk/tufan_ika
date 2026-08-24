#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import serial
import time

class ArduinoMotorKontrol(Node):
    def __init__(self):
        super().__init__('arduino_motor_kontrol_uydusu')
        
        # --- SERİ PORT (USB) BAĞLANTISI ---
        self.seri_port = '/dev/ttyACM0'  # Bağlantı durumuna göre /dev/ttyUSB0 olarak değiştirebilirsiniz
        self.baudrate = 115200 
        self.arduino = None
        self.baglanti_kur()

        # --- PPM AYARLARI ---
        self.ppm_merkez = 1500  # Durma sinyali
        self.ppm_min = 1000     # Tam geri sinyali
        self.ppm_max = 2000     # Tam ileri sinyali

        # --- ABONELİK: Merkezi Yöneticiden Gelen Palet Hızları [-255.0, +255.0] ---
        self.create_subscription(
            Float32MultiArray, 
            '/palet_hizlari', 
            self.palet_callback, 
            10
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

    def baglanti_kur(self):
        try:
            self.arduino = serial.Serial(self.seri_port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Arduino'nun resetlenip kendine gelmesi için bekleme
            self.get_logger().info(f"✅ Arduino ile seri iletişim kuruldu: {self.seri_port}")
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
