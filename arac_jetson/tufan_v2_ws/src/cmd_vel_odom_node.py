#!/usr/bin/env python3
"""Enkoder yokken /cmd_vel'i acik dongu "tekerlek hizi" olarak /wheel/odom'a tasir.

robot_localization EKF bu mesajin sadece twist (vx, vyaw) kismini kullanir;
gercek konum tahmini IMU yaw + bu hizlarin EKF icinde entegrasyonuyla olusur.
Gercek enkoder/odometri kaynagi eklendiginde bu dugum devre disi birakilabilir.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom_node')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._timeout = self.get_parameter('cmd_vel_timeout').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._last_cmd = Twist()
        self._last_cmd_time = None

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self._pub = self.create_publisher(Odometry, 'wheel/odom', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_odom)

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _publish_odom(self):
        now = self.get_clock().now()

        timed_out = (
            self._last_cmd_time is None
            or (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout
        )

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame

        msg.twist.twist = Twist() if timed_out else self._last_cmd

        # Pozu bilmiyoruz; EKF'nin bunu yok saymasi icin buyuk kovaryans veriyoruz.
        msg.pose.covariance[0] = 1e6
        msg.pose.covariance[7] = 1e6
        msg.pose.covariance[35] = 1e6

        msg.twist.covariance[0] = 0.02   # vx
        msg.twist.covariance[7] = 1e6    # vy (kullanilmiyor)
        msg.twist.covariance[35] = 0.05  # vyaw

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
