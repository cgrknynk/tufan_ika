#!/usr/bin/env python3
"""left_wheel_joint/right_wheel_joint icin /joint_states yayinlar.

Bu iki teker (URDF'te 'continuous' tip) gercek enkoder verisi olmadigi
icin hic hareket etmiyor - sadece gorsel/kozmetik. Ama joint_states hic
yayinlanmazsa robot_state_publisher bu tekerlerin TF'ini hic uretemiyor
ve RViz'de RobotModel govdesi "no transform" hatasi veriyordu. Bu dugum
sabit (0.0) aci ile duzenli yayin yaparak o hatayi gideriyor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TekerJointYayinlayici(Node):
    def __init__(self):
        super().__init__('teker_joint_yayinlayici')
        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.1, self._yayinla)

    def _yayinla(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [0.0, 0.0]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TekerJointYayinlayici()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
