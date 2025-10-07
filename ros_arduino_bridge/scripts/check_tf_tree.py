#!/usr/bin/env python3
"""
Diagnostic script to check TF tree for SLAM/mapping issues.
Run this while the robot is operating to verify frame relationships.

Usage:
    ros2 run ros_arduino_bridge check_tf_tree.py
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time


class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for TF buffer to fill
        time.sleep(2.0)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("TF TREE DIAGNOSTIC CHECK")
        self.get_logger().info("=" * 60)
        
        # Expected frames for SLAM
        self.check_transform('map', 'odom')
        self.check_transform('odom', 'base_link')
        self.check_transform('base_link', 'laser')
        self.check_transform('base_link', 'imu_link')
        
        # Check full chain
        self.check_transform('map', 'laser')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("EXPECTED TF TREE STRUCTURE:")
        self.get_logger().info("  map → odom → base_link → laser")
        self.get_logger().info("                        → imu_link")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("IMPORTANT NOTES:")
        self.get_logger().info("  - In RViz, set Fixed Frame to 'map' (NOT 'base_link'!)")
        self.get_logger().info("  - This keeps the map stationary while robot moves")
        self.get_logger().info("  - If map rotates with robot, check Fixed Frame setting")
        self.get_logger().info("=" * 60)
    
    def check_transform(self, parent, child):
        """Check if transform exists between two frames"""
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().info(
                f"✓ {parent} → {child}: "
                f"x={transform.transform.translation.x:.3f} "
                f"y={transform.transform.translation.y:.3f} "
                f"θ={transform.transform.rotation.z:.3f}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"✗ {parent} → {child}: MISSING! ({str(e)})")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = TFChecker()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
