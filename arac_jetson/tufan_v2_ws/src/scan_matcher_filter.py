#!/usr/bin/env python3
"""laser_scan_matcher (konum kaynagi) icin GENIS ACILI, sadece govdeye
bitisik yakin-mesafe yansimalarini temizleyen filtre.

SORUN (canli testte gozlemlendi - araç odometrisi surus sirasinda kayiyor):
laser_scan_matcher, Nav2 costmap'inin de kullandigi 'scan' topic'ini
dinliyordu. O topic (scan_front_filter.py) govdedeki 3D baskinin yan/arka
acilarda urettigi sahte yakin-mesafe yansimalarini (costmap'te temizlenemeyen
"iz" sorununa yol acan asil neden) engellemek icin SADECE ~80 derecelik bir on
koniyi (|ham_aci|>=140) birakip gerisini tamamen 'inf' yapiyor.

Bu, costmap/engel tespiti icin DOGRU bir secim (dar bir on koniyle bile
guvenlik yeterli) ama konum kaynagi (ICP scan-matching) icin agir bir kayip:
canli olcumde, "engellenen" 280 derecelik bolgede bile GERCEK donuslerin
(%20'si >=1.0m, bazilari 8.59m'ye kadar) mevcut oldugu, sadece <0.20m'lik
kisminin (%63) gercekten govde/baski yansimasi oldugu goruldu. Scan-matching
360 derecenin sadece ~%21'i ile calisinca, ozellikle donuslerde referans
noktalari hizla goruş disina cikip ICP'nin dogru yakinsamasi zorlasiyor ->
biriken pozisyon hatasi (surus sirasinda kayma).

COZUM: costmap'in filtresine (scan_front_filter.py -> 'scan') DOKUNMADAN,
scan-matcher icin AYRI bir filtre: sadece govdeye/baskiya GERCEKTEN bitisik
(near_range_m altindaki) noktalari 'inf' yapar, acidan BAGIMSIZ olarak tum
360 derece + uzak donusler korunur. Boylece scan-matcher cok daha zengin bir
nokta bulutuyla calisir, costmap'in guvenlik davranisi hic degismez.

Kullanim: sllidar_node'un ciktisi 'scan_raw'i okuyup 'scan_matcher_filtered'e
yayinlar; laser_scan_matcher bu topic'i dinleyecek sekilde remap edilir
(bkz. launch/tufan_mppi.launch.py).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanMatcherFilter(Node):
    def __init__(self):
        super().__init__('scan_matcher_filter')

        # Canli olcumde govde/baski yansimalari ~0.05m'de kumelenmisti, gercek
        # donusler >=1.0m'de baslıyordu; 0.20m ikisi arasinda guvenli bir pay.
        self.declare_parameter('near_range_m', 0.20)
        self._near = self.get_parameter('near_range_m').value

        self._pub = self.create_publisher(LaserScan, 'scan_matcher_filtered', 10)
        self.create_subscription(LaserScan, 'scan_raw', self._cb, 10)

        self.get_logger().info(
            f'scan_matcher_filter aktif: sadece <{self._near}m donusler temizleniyor, '
            'aci kisitlamasi YOK (scan_front_filter.py aksine, tum 360 derece korunur)'
        )

    def _cb(self, msg: LaserScan):
        yeni_ranges = list(msg.ranges)
        for i, r in enumerate(yeni_ranges):
            if math.isfinite(r) and r < self._near:
                yeni_ranges[i] = float('inf')
        msg.ranges = yeni_ranges
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcherFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
