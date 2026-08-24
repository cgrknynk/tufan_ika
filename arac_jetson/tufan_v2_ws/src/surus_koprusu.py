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
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


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

        self._iz_genisligi = self.get_parameter('iz_genisligi').value
        self._maks_hiz = self.get_parameter('maks_hiz_ms').value
        self._timeout = self.get_parameter('komut_timeout').value
        self._min_pwm = self.get_parameter('min_pwm').value
        self._max_pwm = self.get_parameter('max_pwm').value

        # Guvenli varsayilan: arayuzden mod bilgisi gelene kadar MANUEL kabul et
        # (bilinmeyen durumda otonom surusu asla varsayma).
        self._mod = 'MANUEL'

        self._otonom_twist = Twist()
        self._otonom_zaman = None

        self.create_subscription(String, '/surus_modu', self._mod_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_gated', self._otonom_cb, 10)

        self._palet_pub = self.create_publisher(Float32MultiArray, '/palet_hizlari', 10)

        rate = self.get_parameter('yayin_frekansi').value
        self.create_timer(1.0 / rate, self._dongu)

        self.get_logger().info(
            'surus_koprusu aktif: SADECE OTONOM modda /cmd_vel_gated dinlenip /palet_hizlari '
            'yayinlanir (MANUEL modda tamamen sessiz kalinir, o konuyu yer istasyonu yonetir)'
        )

    def _mod_cb(self, msg: String):
        yeni_mod = msg.data.strip().upper()
        if yeni_mod in ('OTONOM', 'MANUEL') and yeni_mod != self._mod:
            self.get_logger().info(f'surus modu degisti -> {yeni_mod}')
        if yeni_mod in ('OTONOM', 'MANUEL'):
            self._mod = yeni_mod

    def _otonom_cb(self, msg: Twist):
        self._otonom_twist = msg
        self._otonom_zaman = time.time()

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
        sol_pwm = self._deadzone_uygula(sol_ham)
        sag_pwm = self._deadzone_uygula(sag_ham)

        msg = Float32MultiArray()
        msg.data = [float(sol_pwm), float(sag_pwm)]
        self._palet_pub.publish(msg)

    def _deadzone_uygula(self, ham_pwm):
        if abs(ham_pwm) < 1e-6:
            return 0.0
        isaret = 1.0 if ham_pwm > 0 else -1.0
        oran = min(abs(ham_pwm) / 255.0, 1.0)
        olcekli = self._min_pwm + oran * (self._max_pwm - self._min_pwm)
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
