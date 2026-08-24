import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('saf_dinleyici_test')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos)
        print("🟢 Saf Python Abonesi açıldı, veri bekleniyor...")

    def callback(self, msg):
        print(f"✅ VERİ GELDİ! Yakalanan nokta sayısı: {len(msg.ranges)}")

def main():
    rclpy.init()
    node = MinimalSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()