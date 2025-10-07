#!/usr/bin/env python3
"""
Compare raw encoder odometry vs EKF-fused odometry in real-time.
This script shows how much the IMU improves rotation accuracy.

Usage:
    ros2 run ros_arduino_bridge compare_odometry.py
    
Then drive your robot and watch the difference!
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

def quaternion_to_yaw(x, y, z, w):
    """Converts a quaternion into a yaw angle (in degrees)."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(t3, t4)
    return math.degrees(yaw_rad)

class OdometryComparer(Node):
    def __init__(self):
        super().__init__('odometry_comparer')
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0
        
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_yaw = 0.0

        # Subscriber for raw encoder odometry
        self.raw_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.raw_odom_callback,
            10)

        # Subscriber for EKF's filtered odometry
        self.filtered_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_odom_callback,
            10)
        
        self.timer = self.create_timer(0.5, self.print_comparison)
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("ODOMETRY COMPARISON: Raw Encoders vs EKF-Fused")
        self.get_logger().info("=" * 80)
        self.get_logger().info("Drive your robot and watch the difference!")
        self.get_logger().info("The 'Error' shows how much IMU correction is being applied.")
        self.get_logger().info("=" * 80)

    def raw_odom_callback(self, msg):
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.raw_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def filtered_odom_callback(self, msg):
        self.filtered_x = msg.pose.pose.position.x
        self.filtered_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.filtered_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def print_comparison(self):
        # Calculate errors
        x_error = abs(self.filtered_x - self.raw_x)
        y_error = abs(self.filtered_y - self.raw_y)
        yaw_error = abs(self.filtered_yaw - self.raw_yaw)
        
        # Handle wraparound for yaw
        if yaw_error > 180:
            yaw_error = 360 - yaw_error
        
        # Calculate total position error
        position_error = math.sqrt(x_error**2 + y_error**2)
        
        self.get_logger().info(
            f"\n"
            f"┌─────────────────┬──────────┬──────────┬──────────┐\n"
            f"│ Source          │ X (m)    │ Y (m)    │ Yaw (°)  │\n"
            f"├─────────────────┼──────────┼──────────┼──────────┤\n"
            f"│ Raw Encoders    │ {self.raw_x:8.3f} │ {self.raw_y:8.3f} │ {self.raw_yaw:8.2f} │\n"
            f"│ EKF Fused       │ {self.filtered_x:8.3f} │ {self.filtered_y:8.3f} │ {self.filtered_yaw:8.2f} │\n"
            f"├─────────────────┼──────────┼──────────┼──────────┤\n"
            f"│ Error           │ {x_error:8.3f} │ {y_error:8.3f} │ {yaw_error:8.2f} │\n"
            f"└─────────────────┴──────────┴──────────┴──────────┘\n"
            f"Total Position Error: {position_error:.3f}m | Yaw Correction: {yaw_error:.2f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryComparer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
