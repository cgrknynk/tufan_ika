#!/usr/bin/env python3
"""Operatorun verdigi GERCEK DUNYA (lat/lon) hedefini /ugv_goal'a (odom
cercevesi) cevirir - goal_manager_node.py'nin zaten bekledigi format.

NEDEN GEREKLI: yaris/gorev rotasi/hedefleri muhtemelen GPS koordinatlariyla
tarif edilecek, ama /ugv_goal (goal_manager_node.py) SADECE odom-cercevesi
(metre, aracin acilis anindaki KEYFI orijinine gore) x,y kabul ediyor.
Aralarinda hicbir dogal baglanti yok - konum_birlestirici.py'deki GPS
anchor + heading_offset TAM OLARAK bu ceviriyi yapmak icin var (bkz. o
dosyanin basindaki "4) GPS" notu - ayni turetme burada da kullanilir).

*** ONEMLI: anchor VE heading_offset konum_birlestirici.py'de kilitlenir,
BURADA DEGIL - iki node FARKLI anchor/hizalama kullanirsa (ör. herbiri
kendi ilk GPS okumasini anchor sansa) /ugv_goal ile aracin GERCEK GPS
konumu TUTARSIZ olur (goal_manager_node'un dead-reckoning'i ile bu
node'un cevirdigi hedef ayni referansta olmaz). Bu yuzden anchor/
heading_offset burada YENIDEN HESAPLANMAZ, konum_birlestirici.py'nin
yayinladigi /gps_anchor (transient_local - gec baglansak bile ulasir) ve
/gps_heading_offset DOGRUDAN kullanilir.

Giris : /gps_hedef (sensor_msgs/NavSatFix) - operatorun istedigi lat/lon
        (altitude/covariance yok sayilir).
Cikis : /ugv_goal (geometry_msgs/PoseStamped, frame=odom) - yonelim
        varsayilan olarak kimlik (yaw=0) birakilir; send_goal.py'nin CLI
        araciyla ayni varsayilan davranis.

Anchor/heading_offset henuz hazir degilse (GPS henuz RTK'ya konverje
olmadi) gelen hedef SESSIZCE ATILMAZ - bir uyari loglanip yok sayilir;
operator hazir olana kadar beklemeli (bkz. /gps_heading_offset ilk
mesajinin gelmesi = "hizalama hazir" isareti, konum_birlestirici.py notu).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

_EARTH_R = 6378137.0  # bkz. konum_birlestirici.py - AYNI deger, tutarli kalmali


def _latlon_to_enu(lat, lon, lat0, lon0):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * _EARTH_R
    east = dlon * _EARTH_R * math.cos(math.radians(lat0))
    return east, north


def _enu_to_odom(east, north, heading_offset):
    # bkz. konum_birlestirici.py _enu_to_odom - AYNI turetme, tutarli kalmali.
    x = east * math.sin(heading_offset) + north * math.cos(heading_offset)
    y = -east * math.cos(heading_offset) + north * math.sin(heading_offset)
    return x, y


class GpsHedefDonusturucu(Node):
    def __init__(self):
        super().__init__('gps_hedef_donusturucu')

        self._anchor = None  # (lat0, lon0)
        self._heading_offset = None

        anchor_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavSatFix, '/gps_anchor', self._anchor_cb, anchor_qos)
        self.create_subscription(Float64, '/gps_heading_offset', self._heading_cb, 10)
        self.create_subscription(NavSatFix, '/gps_hedef', self._hedef_cb, 10)

        self._pub = self.create_publisher(PoseStamped, '/ugv_goal', 10)

        self.get_logger().info(
            'gps_hedef_donusturucu hazir: /gps_hedef (lat/lon) bekleniyor '
            '(konum_birlestirici.py anchor/hizalamasi hazir olana kadar '
            'gelen hedefler yok sayilir)')

    def _anchor_cb(self, msg: NavSatFix):
        if self._anchor is None:
            self._anchor = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'GPS anchor alindi: lat={msg.latitude:.7f} lon={msg.longitude:.7f}')

    def _heading_cb(self, msg: Float64):
        ilk = self._heading_offset is None
        self._heading_offset = msg.data
        if ilk:
            self.get_logger().info('GPS yon hizalamasi hazir - /gps_hedef artik islenebilir.')

    def _hedef_cb(self, msg: NavSatFix):
        if self._anchor is None or self._heading_offset is None:
            self.get_logger().warn(
                'GPS hedefi alindi ama anchor/hizalama HENUZ HAZIR DEGIL - '
                'yok sayiliyor. (RTK konverje olana kadar bekleyin.)')
            return

        east, north = _latlon_to_enu(msg.latitude, msg.longitude, *self._anchor)
        x, y = _enu_to_odom(east, north, self._heading_offset)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # yaw=0 (kimlik) - send_goal.py ile ayni varsayilan
        self._pub.publish(goal)
        self.get_logger().info(
            f'GPS hedefi -> odom: lat={msg.latitude:.7f} lon={msg.longitude:.7f} '
            f'-> x={x:.2f} y={y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsHedefDonusturucu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
