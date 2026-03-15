#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanHz(Node):
    def __init__(self):
        super().__init__('scan_hz')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(LaserScan, '/scan_bag', self.cb, qos)

        self.count = 0
        self.t0 = time.time()
        self.last_print = self.t0

    def cb(self, msg):
        self.count += 1
        now = time.time()
        if now - self.last_print >= 1.0:
            dt = now - self.t0
            hz = self.count / dt if dt > 0 else 0.0
            self.get_logger().info(f"/scan_bag rate ~ {hz:.2f} Hz   (msgs={self.count}, dt={dt:.1f}s)")
            self.last_print = now

def main():
    rclpy.init()
    node = ScanHz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
