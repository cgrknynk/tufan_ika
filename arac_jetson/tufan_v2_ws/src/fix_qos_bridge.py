#!/usr/bin/env python3
"""mavros'un /mavros/global_position/raw/fix (NavSatFix, BEST_EFFORT) yayinini
/fix (NavSatFix, RELIABLE) olarak yeniden yayinlar - arayuz tarafindaki
tufan_gcs_master_node /fix'i RELIABLE QoS ile dinliyor, mavros'un
BEST_EFFORT yayiniyla DDS seviyesinde hic eslesmiyordu (QoS uyumsuzlugu,
mesaj gitmiyordu). Arayuz kodu degistirilmeden koprulemek icin bu node
araya konuldu. raw/fix (dogrudan GPS_RAW_INT'ten, Here4'un ham/RTK
duzeltilmis fix'i) kullanilir - global_position/global (ArduPilot'un
kendi EKF ile IMU+GPS harmanlamis tahmini) DEGIL, cunku EKF'in
konverje olmasina bagli degil ve asil ihtiyac zaten ham GPS konumu.

TASINDI (ayri/elle calistirilan here4_gnss/fix_qos_bridge.py'den):
artik tufan_v2_ws paketinin bir parcasi, launch/gps_bringup.launch.py ile
otomatik baslar - elle, workspace disindan calistirmaya gerek yok.

NOT: konum_birlestirici.py ve gps_hedef_donusturucu.py bu koprudeki /fix'i
KULLANMAZ - kendi GPS ihtiyaclari icin /mavros/global_position/raw/fix'e
DOGRUDAN, kendi BEST_EFFORT uyumlu QoS'uyla abone olurlar (bkz. o
dosyalarin basi). Bu kopru SADECE arayuz/GCS tarafindaki eski RELIABLE-
bekleyen node icin var.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix


class FixQosBridge(Node):
    def __init__(self):
        super().__init__('fix_qos_bridge')

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub = self.create_publisher(NavSatFix, '/fix', reliable)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self._on_fix, best_effort
        )
        self.get_logger().info(
            '/mavros/global_position/raw/fix (BEST_EFFORT) -> /fix (RELIABLE) koprusu aktif'
        )

    def _on_fix(self, msg):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FixQosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
