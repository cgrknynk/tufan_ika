#!/usr/bin/env python3
"""rf2o (LIDAR) VE IMU verisini "minimum hareket" filtresiyle EKF'ye aktarir.

Ilk deneme /palet_hizlari (komut) sinyaline bakarak "hareket var/yok" karari
veriyordu - ama arac GERCEKTE hareket etmedigi halde RViz'de surekli kaymaya
devam etti (komut sinyali guvenilir bir "gercekten duruyor" gostergesi degil).

Bu surum komuta hic bakmaz, SENSORLERIN KENDI olcumune bakar:

1) LIDAR (rf2o, /odom_rf2o -> /odom_rf2o_gated): kabul edilen konumla yeni
   rf2o okumasi arasindaki fark (mesafe + aci) kucuk bir esigin altindaysa
   scan-matching gurultusu sayilip yok sayilir (konum SABIT, hiz 0 yayinlanir
   - EKF'ye acikca "hiz=0" (ZUPT) verilir). Esik asilirsa gercek hareket kabul
   edilir.

2) IMU (BNO055, NDOF modu mutlak yon veriyor ama manyetometre kalibrasyonu/
   motor girisimi yuzunden DURURKEN bile hafifce gezinebilir - /imu/data ->
   /imu/data_gated): kabul edilen mutlak yon (quaternion) ile yeni IMU
   okumasi arasindaki aci farki kucuk bir esigin altindaysa yon SABIT tutulur
   (acisal hiz/ivme oldugu gibi gecer, sadece mutlak yon dondurulur). Esik
   asilirsa gercek donus kabul edilir.

EKF, /odom_rf2o ve /imu/data yerine bu dugumun uretecegi
/odom_rf2o_gated ve /imu/data_gated'i dinlemeli.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def _yaw_al(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _aci_farki(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class OdomHareketKapisi(Node):
    def __init__(self):
        super().__init__('odom_hareket_kapisi')

        # --- LIDAR (rf2o) esikleri ---
        self.declare_parameter('min_hareket_mesafesi', 0.03)  # metre
        self.declare_parameter('min_hareket_acisi_derece', 2.0)  # derece
        self._min_mesafe = self.get_parameter('min_hareket_mesafesi').value
        self._min_aci = math.radians(self.get_parameter('min_hareket_acisi_derece').value)

        # --- IMU esigi ---
        self.declare_parameter('min_imu_acisi_derece', 0.5)  # derece
        self._min_imu_aci = math.radians(self.get_parameter('min_imu_acisi_derece').value)

        self._kabul_x = None
        self._kabul_y = None
        self._kabul_yaw = None
        self._kabul_pose = None

        self._kabul_imu_yaw = None
        self._kabul_imu_orientation = None

        self.create_subscription(Odometry, '/odom_rf2o', self._odom_cb, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom_rf2o_gated', 10)

        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data_gated', 10)

        self.get_logger().info(
            'odom_hareket_kapisi aktif: LIDAR '
            f'{self._min_mesafe}m/{self.get_parameter("min_hareket_acisi_derece").value} derece, '
            f'IMU {self.get_parameter("min_imu_acisi_derece").value} derece '
            'altindaki degisimler gurultu sayilip yok sayilir'
        )

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_al(msg.pose.pose.orientation)

        if self._kabul_pose is None:
            self._kabul_x, self._kabul_y, self._kabul_yaw = x, y, yaw
            self._kabul_pose = msg.pose
            self._odom_pub.publish(msg)
            return

        mesafe = math.hypot(x - self._kabul_x, y - self._kabul_y)
        dyaw = _aci_farki(yaw, self._kabul_yaw)

        if mesafe < self._min_mesafe and dyaw < self._min_aci:
            # Gurultu: kabul edilen konumu SABIT tut, hizi acikca sifirla
            # (EKF'nin vx/vy fuzyonu bunu dogrudan "sifir hiz" olcumu olarak
            # kullanir - standart ZUPT teknigi, hareket bittikten sonra
            # EKF'nin ic hiz tahmininin yavasca sifira surunmesini onler).
            cikis = Odometry()
            cikis.header = msg.header
            cikis.child_frame_id = msg.child_frame_id
            cikis.pose = self._kabul_pose
            cikis.twist.twist.linear.x = 0.0
            cikis.twist.twist.linear.y = 0.0
            cikis.twist.twist.angular.z = 0.0
            cikis.twist.covariance = msg.twist.covariance
            self._odom_pub.publish(cikis)
            return

        self._kabul_x, self._kabul_y, self._kabul_yaw = x, y, yaw
        self._kabul_pose = msg.pose
        self._odom_pub.publish(msg)

    def _imu_cb(self, msg: Imu):
        yaw = _yaw_al(msg.orientation)

        if self._kabul_imu_orientation is None:
            self._kabul_imu_yaw = yaw
            self._kabul_imu_orientation = msg.orientation
            self._imu_pub.publish(msg)
            return

        dyaw = _aci_farki(yaw, self._kabul_imu_yaw)

        if dyaw < self._min_imu_aci:
            # Gurultu: mutlak yonu SABIT tut. Acisal hiz/ivme oldugu gibi
            # gecer (bunlar ham anlik olcum, dondurmeye gerek yok).
            cikis = Imu()
            cikis.header = msg.header
            cikis.orientation = self._kabul_imu_orientation
            cikis.orientation_covariance = msg.orientation_covariance
            cikis.angular_velocity = msg.angular_velocity
            cikis.angular_velocity_covariance = msg.angular_velocity_covariance
            cikis.linear_acceleration = msg.linear_acceleration
            cikis.linear_acceleration_covariance = msg.linear_acceleration_covariance
            self._imu_pub.publish(cikis)
            return

        self._kabul_imu_yaw = yaw
        self._kabul_imu_orientation = msg.orientation
        self._imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomHareketKapisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
