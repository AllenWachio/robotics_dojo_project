#!/usr/bin/env python3
"""
Initial Pose Publisher for Nav2
Automatically publishes initial pose to AMCL on startup
This allows Nav2 to start without manual intervention in RViz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Declare parameters for initial pose (can be overridden in launch file)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_delay', 2.0)  # Wait for Nav2 to start
        self.declare_parameter('publish_count', 3)     # Publish multiple times to ensure AMCL receives it
        
        # Get parameters
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_z = self.get_parameter('initial_z').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        self.publish_delay = self.get_parameter('publish_delay').value
        self.publish_count = self.get_parameter('publish_count').value
        
        # QoS profile for initial pose (transient local for reliability)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publisher for initial pose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )
        
        self.get_logger().info(f'Initial Pose Publisher started')
        self.get_logger().info(f'Will publish initial pose after {self.publish_delay}s delay')
        self.get_logger().info(f'Initial position: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}')
        
        # Wait before publishing (let Nav2 nodes start)
        self.timer = self.create_timer(self.publish_delay, self.publish_initial_pose)
        self.publish_counter = 0
        
    def publish_initial_pose(self):
        """Publish the initial pose to AMCL"""
        if self.publish_counter >= self.publish_count:
            self.get_logger().info(f'Published initial pose {self.publish_count} times. Shutting down.')
            self.timer.cancel()
            return
            
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = self.initial_z
        
        # Convert yaw to quaternion
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        
        # Set covariance (uncertainty in initial pose)
        # Diagonal: x, y, z, roll, pitch, yaw
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance = 0.25 (0.5m std dev)
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y variance = 0.25 (0.5m std dev)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z (not used for 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll (not used for 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # pitch (not used for 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068  # yaw variance = 0.068 (15° std dev)
        ]
        
        self.publisher.publish(msg)
        self.publish_counter += 1
        self.get_logger().info(f'Published initial pose ({self.publish_counter}/{self.publish_count})')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
