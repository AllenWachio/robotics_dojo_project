#!/usr/bin/env python3
"""
SLAM Diagnostics Script for ROS Arduino Bridge
This script helps diagnose common SLAM issues by checking:
- TF transforms
- Topic publications
- Node status
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time

class SLAMDiagnostics(Node):
    def __init__(self):
        super().__init__('slam_diagnostics')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Track message reception
        self.scan_received = False
        self.odom_received = False
        self.last_scan_time = None
        self.last_odom_time = None
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for periodic diagnostics
        self.timer = self.create_timer(5.0, self.run_diagnostics)
        
        self.get_logger().info("SLAM Diagnostics node started")
        self.checks_run = 0

    def scan_callback(self, msg):
        self.scan_received = True
        self.last_scan_time = self.get_clock().now()
        
    def odom_callback(self, msg):
        self.odom_received = True
        self.last_odom_time = self.get_clock().now()

    def run_diagnostics(self):
        self.checks_run += 1
        self.get_logger().info(f"\n=== SLAM Diagnostics Check #{self.checks_run} ===")
        
        # Check topics
        self.check_topics()
        
        # Check TF transforms
        self.check_transforms()
        
        # Summary
        self.get_logger().info("=== End Diagnostics ===\n")

    def check_topics(self):
        """Check if essential topics are being published"""
        self.get_logger().info("Checking topics:")
        
        current_time = self.get_clock().now()
        
        # Check scan topic
        if self.scan_received and self.last_scan_time:
            scan_age = (current_time - self.last_scan_time).nanoseconds / 1e9
            if scan_age < 2.0:
                self.get_logger().info("  ✓ /scan: ACTIVE (last: {:.1f}s ago)".format(scan_age))
            else:
                self.get_logger().warn("  ⚠ /scan: OLD DATA (last: {:.1f}s ago)".format(scan_age))
        else:
            self.get_logger().error("  ✗ /scan: NO DATA RECEIVED")
        
        # Check odometry topic
        if self.odom_received and self.last_odom_time:
            odom_age = (current_time - self.last_odom_time).nanoseconds / 1e9
            if odom_age < 2.0:
                self.get_logger().info("  ✓ /odom: ACTIVE (last: {:.1f}s ago)".format(odom_age))
            else:
                self.get_logger().warn("  ⚠ /odom: OLD DATA (last: {:.1f}s ago)".format(odom_age))
        else:
            self.get_logger().error("  ✗ /odom: NO DATA RECEIVED")

    def check_transforms(self):
        """Check if essential transforms are available"""
        self.get_logger().info("Checking transforms:")
        
        transforms_to_check = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'laser'),
            ('base_link', 'lidar_1')
        ]
        
        for parent, child in transforms_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                self.get_logger().info(f"  ✓ {parent} -> {child}: OK")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"  ⚠ {parent} -> {child}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    diagnostics = SLAMDiagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()