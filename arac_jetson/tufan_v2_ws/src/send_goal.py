#!/usr/bin/env python3
"""Basit hedef gönderme aracı.

GERCEK ARAC PORTU (kaynak: ros2_ws/src/tufan_simulation/scripts/send_goal.py).
Tek fark: hedef frame'i 'map' yerine 'odom' (bkz. goal_manager_node.py basi).

goal_manager_node'a (/ugv_goal) bir hedef yayınlar; bu node hedefi, haritasız
costmap penceresinin sınırlarını aşmayacak ara "bacaklar" halinde otonom
olarak Nav2 MPPI denetleyicisine iletir. Uzaklık ne olursa olsun çalışır.

Kullanım:
    ros2 run tufan_v2_ws send_goal.py <x> <y> [yaw_deg]
"""
import sys
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String


class SendGoalNode(Node):

    def __init__(self, x, y, yaw):
        super().__init__('send_goal_cli')
        self._pub = self.create_publisher(PoseStamped, '/ugv_goal', 10)
        self.create_subscription(Float32, '/ugv_goal_feedback', self._on_feedback, 10)
        self.create_subscription(String, '/ugv_goal_result', self._on_result, 10)
        self.done = False

        # DDS keşfi (discovery) her zaman anlık değildir: yayıncı hazır
        # olur olmaz göndermek, abone henüz eşleşmeden mesajın kaybolmasına
        # yol açabilir (bu ortamda gözlemlendi). Bu yüzden gerçekten en az
        # bir abone eşleşene KADAR bekleyip, ardından güvenlik için birkaç
        # kez daha tekrar gönderiyoruz.
        self._remaining_sends = 5
        self._x, self._y, self._yaw = x, y, yaw
        self._timer = self.create_timer(0.2, self._send_once)

    def _send_once(self):
        if self._remaining_sends <= 0:
            self._timer.cancel()
            return
        if self._pub.get_subscription_count() < 1:
            return  # abone henuz eslesmedi, bu tick'i atla
        self._remaining_sends -= 1
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.orientation.z = math.sin(self._yaw / 2.0)
        msg.pose.orientation.w = math.cos(self._yaw / 2.0)
        self._pub.publish(msg)
        print(f'Hedef gonderildi: x={self._x} y={self._y} '
              f'yaw={math.degrees(self._yaw):.1f} deg')

    def _on_feedback(self, msg: Float32):
        print(f'  kalan mesafe: {msg.data:.2f} m', end='\r')

    def _on_result(self, msg: String):
        print()
        if msg.data == 'SUCCESS':
            print('Hedefe ulasildi.')
        elif msg.data == 'CANCELLED':
            print('Gorev iptal edildi.')
        else:
            print('Gorev basarisiz oldu.')
        self.done = True


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = SendGoalNode(x, y, yaw)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
