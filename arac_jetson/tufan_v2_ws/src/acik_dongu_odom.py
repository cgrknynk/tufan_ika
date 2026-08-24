#!/usr/bin/env python3
"""Acik dongu (open-loop) konum tahmini - sensor gurultusu YOK.

Gercek teker enkoderi/GPS yok. LIDAR tabanli odometri (rf2o) uc kez denendi,
her seferinde arac DURURKEN bile scan-matching gurultusu yuzunden RViz'de
kayma/drift devam etti - kesin olarak reddedildi.

Bu dugum FARKLI bir yaklasim kullanir: hicbir sensor olcumune (LIDAR/IMU'nun
kendi konum tahminine) GUVENMEZ. Sadece motorlara GONDERILEN GERCEK komutu
(bkz. /palet_hizlari - surus_koprusu'nun nihai karari, guvenlik durusu ve
mod secimi dahil edilmis) matematiksel olarak zaman icinde entegre eder:

  - /palet_hizlari sifir ise (arac gercekten duruyor - MANUEL modda sessiz,
    OTONOM'da hedef yok, ya da engel guvenlik durusu aktif): entegrasyon da
    MATEMATIKSEL OLARAK sifirdir. Kayma FIZIKSEL OLARAK IMKANSIZ - gurultu
    diye bir sey yok, ortada islenecek sensor verisi yok.
  - /palet_hizlari sifir degilse (arac gercekten hareket komutu aliyor):
    /cmd_vel'deki (nav2/MPPI ciktisi) hiz, gercek IMU yon (yaw) yonunde
    entegre edilir.

Bu YAKLASIK bir tahmindir (gercek tekerlek kaymasi/surtunme payi hesaba
katilmaz, GPS/enkoder hassasiyetinde DEGILDIR) ama durgun haldeyken kayma
GARANTI OLARAK sifirdir - bu, kullanicinin oncelikli talebiydi.

EKF'ye /odom_acik_dongu olarak (sadece x,y konum + vx,vy) girdi saglar; yon
(yaw) hala ayri olarak gercek IMU'dan (/imu/data) fuzyonlanir.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


class AcikDonguOdom(Node):
    def __init__(self):
        super().__init__('acik_dongu_odom')

        self.declare_parameter('komut_timeout', 0.5)
        self.declare_parameter('yayin_frekansi', 20.0)
        self._timeout = self.get_parameter('komut_timeout').value

        self._x = 0.0
        self._y = 0.0
        self._imu_yaw = 0.0
        self._imu_orientation = None

        self._cmd_vel = Twist()
        self._cmd_zaman = None
        self._palet_aktif = False

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Float32MultiArray, '/palet_hizlari', self._palet_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        self._pub = self.create_publisher(Odometry, '/odom_acik_dongu', 10)

        rate = self.get_parameter('yayin_frekansi').value
        self._son_zaman = time.time()
        self.create_timer(1.0 / rate, self._dongu)

        self.get_logger().info(
            'acik_dongu_odom aktif: sadece /palet_hizlari gercekten hareket '
            'komutu tasirken /cmd_vel + gercek IMU yon entegre edilir - '
            'durgun haldeyken kayma matematiksel olarak imkansiz'
        )

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_vel = msg
        self._cmd_zaman = time.time()

    def _palet_cb(self, msg: Float32MultiArray):
        self._palet_aktif = any(abs(v) > 1e-6 for v in msg.data)

    def _imu_cb(self, msg: Imu):
        self._imu_orientation = msg.orientation
        siny_cosp = 2.0 * (msg.orientation.w * msg.orientation.z
                            + msg.orientation.x * msg.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (msg.orientation.y ** 2 + msg.orientation.z ** 2)
        self._imu_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _dongu(self):
        simdi = time.time()
        dt = simdi - self._son_zaman
        self._son_zaman = simdi

        cmd_taze = self._cmd_zaman is not None and (simdi - self._cmd_zaman) < self._timeout
        v = self._cmd_vel.linear.x if (self._palet_aktif and cmd_taze) else 0.0
        w = self._cmd_vel.angular.z if (self._palet_aktif and cmd_taze) else 0.0

        # Sadece gercekten hareket komutu varken entegre et - v=0 iken bu
        # zaten hicbir sey degistirmez (0*dt=0), ayri bir "dur" kolu gerekmez.
        self._x += v * math.cos(self._imu_yaw) * dt
        self._y += v * math.sin(self._imu_yaw) * dt

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        if self._imu_orientation is not None:
            msg.pose.pose.orientation = self._imu_orientation
        else:
            msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AcikDonguOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
