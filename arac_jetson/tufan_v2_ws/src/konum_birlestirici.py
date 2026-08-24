#!/usr/bin/env python3
"""Konum artik ICP (/odom_lsm) DEGIL, hiz+yon ENTEGRASYONU (dead-reckoning) ile
hesaplaniyor: konum(x,y)=rf2o hizi + IMU yonu entegre edilerek, yon=IMU, EKF YOK.

Bu mimari, canli testte tespit edilen UC ayri sorunun sirayla arastirilip
duzeltilmesinin sonucudur:

1) YAW: /imu/data'nin yawi BNO055 manyetometresi/ivmeolceri kalibrasyonsuz
   oldugu icin (calib_status: accel=0 mag=0) motor akiminda 13.6s'de 2.14
   derece sicradi, /odom_lsm'in kendi (ICP) yawi ayni sartlarda 48s'de sadece
   0.2 derece sapti. Once yaw da LSM'e tasindi ama kullanicinin tercihiyle
   IMU'ya GERI DONULDU (BNO055'in gyro ekseni zaten kalibre - 3/3 - ve
   manyetik girisimden etkilenmiyor). Acisal hiz de (angular_velocity.z) ayni
   nedenle dogrudan gyro'dan alinir. BU KISIM CANLI TESTTE IYI SONUC VERDI,
   DEGISTIRILMEDI.

2) LINEER HIZ: laser_scan_matcher.cpp'nin twist'i hiz=pozisyon_farki/dt olarak
   hesapliyordu, dt GERCEK ZAMANLI (wall-clock) araligiydi - Jetson'da Nav2
   yigini CPU paylasirken ufak bir zamanlama sapmasi bile FIZIKSEL OLARAK
   IMKANSIZ hiz sicramalarina cevirdi. Cozum: rf2o_laser_odometry - hizi
   range-flow denklemleriyle DOGRUDAN hesaplayan (Jaimez & Gonzalez-Jimenez),
   kendi ic zaman farkini LIDAR donanim zaman damgasindan alan, son 4
   orneğin ortalamasiyla zaten yumusatilmis bir kaynak. BU KISIM DA IYI
   SONUC VERDI, DEGISTIRILMEDI.

3) KONUM (x,y) - ASIL SORUN: /odom_lsm (laser_scan_matcher PL-ICP) canli
   testte "sag-sol/donus cok iyi ama ileri-geri hic yok" seklinde basarisiz
   oldu - 3 dakikalik gercek surus boyunca x,y toplam 15cm'den az degisti.
   Sahne incelemesi net bir "aperture problem" gosterdi: aracin onu sadece
   dar bir koniyle (140 derece esik, ~80 derece acik alan - govdedeki 3D
   baski yuzunden) goruyor, tam karsida 13.4m'de neredeyse TAMAMEN DUZ/SABIT
   bir yuzey var (10 derecelik dilimde sadece 1cm fark - ileri hareketi
   ayirt etmek icin pratikte bilgisiz), konun kenarlarinda ise degisken
   (2-4m) yakin yapi var - PL-ICP bu dar/dejenere geometride donusu/yanal
   kaymayi yakalayabiliyor ama ileri mesafeyi neredeyse hic kestiremiyor.
   FOV'u genisletmek DAHA ONCE DENENDI ve ICP'yi TAMAMEN BOZDU (bkz.
   launch dosyasindaki scan_matcher_filter_node yorumu). Bu yuzden ICP'ye
   guvenmek yerine KONUM ARTIK KLASIK "dead-reckoning" ILE HESAPLANIYOR:
   rf2o'nun (adim 2'de zaten guvenilir oldugu kanitlanan) lineer hizi,
   IMU'nun (adim 1'de zaten guvenilir oldugu kanitlanan) yonune izdusurulup
   zaman icinde entegre edilir (x += vx*cos(yaw)*dt, y += vx*sin(yaw)*dt).
   Bu, ICP'nin dar-FOV dejenerasyonunu TAMAMEN ATLAR (rf2o'nun range-flow
   yontemi, PL-ICP'nin aksine seyrek nokta eslesmesine dayanmadigi icin bu
   ozel dejenere geometriye karsi cok daha dayanikli - canli testte hiz
   cikti degerleri surekli/tepkiseldi). Bilinen dezavantaj: saf entegrasyon
   (tekerlek/wheel-odometri gibi) sinirsiz surunme (drift) biriktirebilir -
   ICP'nin ortam-bagimsiz kendi kendini duzeltmesi yok - ama mevcut
   durumda ICP zaten pratikte SIFIR ileri bilgi verdigi icin bu KESIN BIR
   IYILESTIRME. /odom_lsm artik HIC KULLANILMIYOR.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KonumBirlestirici(Node):
    def __init__(self):
        super().__init__('konum_birlestirici')

        self.declare_parameter('integrate_rate_hz', 20.0)
        # CANLI TESTTE BULUNDU: rf2o'nun twist.linear.x isareti bu montajda
        # GERCEK ileri yonunun TERSI cikti (geri surunce RViz'de ileri
        # gidiyor gibi gorundu, ve tam tersi). Yon/donus (IMU kaynakli)
        # DOGRUYDU - sadece hiz isareti ters. Tek noktadan duzeltiliyor.
        self.declare_parameter('vx_sign', -1.0)
        self._vx_sign = self.get_parameter('vx_sign').value

        self._son_imu_orientation = None
        self._son_imu_wz = 0.0
        self._son_vx = 0.0

        # dead-reckoning durumu (odom cercevesinde, baslangic = 0,0)
        self._x = 0.0
        self._y = 0.0
        self._last_tick_time = None

        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self._rf2o_cb, 10)

        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        rate = self.get_parameter('integrate_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'konum_birlestirici aktif: EKF YOK. Konum(x,y)=rf2o hizi + IMU '
            'yonu entegre edilerek (dead-reckoning) hesaplaniyor (ICP/odom_lsm '
            'ARTIK KULLANILMIYOR), yon+acisal_hiz=/imu/data (bkz. dosya basi notu)'
        )

    def _imu_cb(self, msg: Imu):
        self._son_imu_orientation = msg.orientation
        self._son_imu_wz = msg.angular_velocity.z

    def _rf2o_cb(self, msg: Odometry):
        self._son_vx = self._vx_sign * msg.twist.twist.linear.x

    def _tick(self):
        if self._son_imu_orientation is None:
            return  # henuz gercek yon bilgisi yok, entegrasyona baslama

        now = self.get_clock().now()
        if self._last_tick_time is None:
            self._last_tick_time = now
            return
        dt = (now - self._last_tick_time).nanoseconds / 1e9
        self._last_tick_time = now
        if dt <= 0.0 or dt > 0.5:
            # ilk tik / anormal buyuk bosluk (ör. node donmustu): bu adimi
            # entegre etme, sadece zaman damgasini guncelle.
            return

        yaw = _yaw_of(self._son_imu_orientation)
        self._x += self._son_vx * math.cos(yaw) * dt
        self._y += self._son_vx * math.sin(yaw) * dt

        stamp = now.to_msg()

        cikis = Odometry()
        cikis.header.stamp = stamp
        cikis.header.frame_id = 'odom'
        cikis.child_frame_id = 'base_footprint'
        cikis.pose.pose.position.x = self._x
        cikis.pose.pose.position.y = self._y
        cikis.pose.pose.position.z = 0.0
        cikis.pose.pose.orientation = self._son_imu_orientation
        cikis.twist.twist.linear.x = self._son_vx
        cikis.twist.twist.angular.z = self._son_imu_wz
        self._pub.publish(cikis)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self._son_imu_orientation
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KonumBirlestirici()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
