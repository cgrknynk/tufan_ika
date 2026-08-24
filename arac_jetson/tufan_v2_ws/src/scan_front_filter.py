#!/usr/bin/env python3
"""LIDAR govdesindeki 3D baski kapatma parcasi, sensorun cogu aciya (yan/arka)
sensore bitisik sahte yakin-mesafe yansimalari uretiyor. obstacle_min_range
bunun bir kismini filtreler ama garanti degil (baski farkli mesafelerde de
yansitabilir). Bu dugum daha kesin bir cozum: SADECE dogrulanmis acik on
koniyi birakir, geri kalan tum acilari 'inf' (engel yok) yapar.

TF: base_footprint -> lidar_link su an net 180 derece donuk (robot.xacro).
Bu yuzden ham scan'in +-180 derece civari (angle_min/angle_max sinirlarina
yakin) GERCEK ONDUR; 0 derece civari gercek ARKADIR (baski ile kapali).
Fiziksel montaj degisirse (baski/LIDAR yeniden konumlandirilirsa) bu esik
degerinin yeniden dogrulanmasi gerekir.

Kullanim: sllidar_node'un ciktisi 'scan_raw'a remap edilir, bu dugum
'scan_raw'i okuyup filtrelenmis veriyi 'scan'e (nav2'nin kullandigi asil
topic) yayinlar.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrontFilter(Node):
    def __init__(self):
        super().__init__('scan_front_filter')

        # |raw_aci| (derece, -180..180 normalize) bu esikten BUYUK/ESIT ise acik
        # on koni sayilir ve veri korunur; kucukse (yan/arka, baskili) 'inf' yapilir.
        self.declare_parameter('acik_esik_derece', 140.0)
        self.esik_rad = math.radians(self.get_parameter('acik_esik_derece').value)

        self._pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(LaserScan, 'scan_raw', self._cb, 10)

        self.get_logger().info(
            f'scan_front_filter aktif: |ham_aci| >= {self.get_parameter("acik_esik_derece").value}'
            ' derece disindaki (yan/arka, 3D baski ile kapali) okumalar inf yapiliyor'
        )

    def _cb(self, msg: LaserScan):
        yeni_ranges = list(msg.ranges)
        n = len(yeni_ranges)
        for i in range(n):
            aci = msg.angle_min + i * msg.angle_increment
            aci = math.atan2(math.sin(aci), math.cos(aci))  # -pi..pi normalize
            if abs(aci) < self.esik_rad:
                yeni_ranges[i] = float('inf')

        msg.ranges = yeni_ranges
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrontFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
