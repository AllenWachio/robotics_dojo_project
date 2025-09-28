#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/scan_throttled', 10)
        self.last_publish_time = self.get_clock().now()
        self.rate = 5.0  # Throttle to 5 Hz
        self.get_logger().info(f"Throttling /scan to {self.rate} Hz")

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        if (current_time - self.last_publish_time).nanoseconds / 1e9 >= 1.0 / self.rate:
            self.publisher.publish(msg)
            self.last_publish_time = current_time

def main(args=None):
    rclpy.init(args=args)
    scan_throttle = ScanThrottle()
    rclpy.spin(scan_throttle)
    scan_throttle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
