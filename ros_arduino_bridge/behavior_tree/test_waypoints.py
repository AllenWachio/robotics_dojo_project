#!/usr/bin/env python3
"""
Waypoint Testing and Visualization Tool
========================================
Test individual waypoints before running full mission

Usage:
    python3 test_waypoints.py list           # Show all waypoints
    python3 test_waypoints.py goto <name>    # Navigate to specific waypoint
    python3 test_waypoints.py test_all       # Test navigation to all waypoints
    python3 test_waypoints.py measure        # Help measure actual coordinates

Author: Robotics Dojo 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import sys
import math
import time

# Import waypoints from competition mission
from competition_mission import WAYPOINTS


class WaypointTester(Node):
    def __init__(self):
        super().__init__('waypoint_tester')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriber for current pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.current_pose = None
        
        self.get_logger().info('Waypoint Tester initialized')
    
    def pose_callback(self, msg):
        """Store current robot pose"""
        self.current_pose = msg.pose.pose
    
    def navigate_to_waypoint(self, waypoint_name):
        """Navigate to a specific waypoint"""
        if waypoint_name not in WAYPOINTS:
            self.get_logger().error(f"Unknown waypoint: {waypoint_name}")
            return False
        
        x, y = WAYPOINTS[waypoint_name]
        
        self.get_logger().info(f"Navigating to {waypoint_name}: ({x}, {y})")
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return False
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(x, y)
        
        # Send goal
        send_future = self.nav_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
        
        self.get_logger().info("Goal accepted, navigating...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"✓ Reached {waypoint_name}!")
            return True
        else:
            self.get_logger().error(f"Navigation failed with status {result.status}")
            return False
    
    def make_pose_stamped(self, x, y, yaw=0.0):
        """Create PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def get_current_position(self):
        """Get current robot position"""
        if self.current_pose is None:
            self.get_logger().warn("No pose data yet, waiting...")
            
            # Wait up to 5 seconds for pose
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.current_pose is not None:
                    break
            
            if self.current_pose is None:
                self.get_logger().error("Could not get current pose!")
                return None
        
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        return (x, y)


def list_waypoints():
    """Print all waypoints"""
    print("\n" + "="*60)
    print("COMPETITION WAYPOINTS")
    print("="*60)
    for name, (x, y) in WAYPOINTS.items():
        print(f"  {name:20s} → ({x:6.2f}, {y:6.2f})")
    print("="*60 + "\n")


def measure_mode():
    """Help user measure waypoint coordinates"""
    print("\n" + "="*60)
    print("WAYPOINT MEASUREMENT GUIDE")
    print("="*60)
    print("\nMethod 1: RViz (Recommended)")
    print("-" * 60)
    print("1. Launch navigation:")
    print("   ros2 launch ros_arduino_bridge laptop_navigation.launch.py")
    print("\n2. Open RViz")
    print("\n3. Use '2D Nav Goal' tool to click on map")
    print("\n4. Watch terminal for goal coordinates like:")
    print("   [navigate_to_pose] Received goal: (x=1.23, y=0.45)")
    print("\n5. Write down coordinates for each waypoint")
    
    print("\n\nMethod 2: Current Pose Topic")
    print("-" * 60)
    print("1. Drive robot to waypoint location (teleop)")
    print("\n2. Check current position:")
    print("   ros2 topic echo /amcl_pose")
    print("\n3. Record pose.pose.position.x and .y values")
    
    print("\n\nMethod 3: Use This Script")
    print("-" * 60)
    print("1. Make sure navigation is running")
    print("\n2. Run:")
    print("   python3 test_waypoints.py current")
    print("\n3. Drive robot to each waypoint and run command")
    print("="*60 + "\n")


def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("\nUsage:")
        print("  python3 test_waypoints.py list              - Show all waypoints")
        print("  python3 test_waypoints.py goto <waypoint>   - Navigate to waypoint")
        print("  python3 test_waypoints.py test_all          - Test all waypoints")
        print("  python3 test_waypoints.py current           - Show current position")
        print("  python3 test_waypoints.py measure           - Measurement guide")
        print("\nAvailable waypoints:")
        for name in WAYPOINTS.keys():
            print(f"  - {name}")
        print()
        return
    
    command = sys.argv[1].lower()
    
    if command == 'list':
        list_waypoints()
    
    elif command == 'measure':
        measure_mode()
    
    elif command in ['goto', 'test_all', 'current']:
        # Initialize ROS
        rclpy.init()
        tester = WaypointTester()
        
        try:
            if command == 'goto':
                if len(sys.argv) < 3:
                    print("Usage: python3 test_waypoints.py goto <waypoint_name>")
                    list_waypoints()
                    return
                
                waypoint_name = sys.argv[2]
                tester.navigate_to_waypoint(waypoint_name)
            
            elif command == 'current':
                print("\nGetting current position...")
                pos = tester.get_current_position()
                if pos:
                    x, y = pos
                    print(f"\n{'='*60}")
                    print(f"CURRENT POSITION: ({x:.3f}, {y:.3f})")
                    print(f"{'='*60}")
                    
                    # Find closest waypoint
                    min_dist = float('inf')
                    closest = None
                    for name, (wx, wy) in WAYPOINTS.items():
                        dist = math.sqrt((x - wx)**2 + (y - wy)**2)
                        if dist < min_dist:
                            min_dist = dist
                            closest = name
                    
                    if closest:
                        print(f"\nClosest waypoint: {closest} (distance: {min_dist:.2f}m)")
                    print()
            
            elif command == 'test_all':
                print("\n" + "="*60)
                print("TESTING ALL WAYPOINTS")
                print("="*60 + "\n")
                
                results = {}
                for name in WAYPOINTS.keys():
                    print(f"\nTesting: {name}")
                    print("-" * 40)
                    success = tester.navigate_to_waypoint(name)
                    results[name] = success
                    
                    if success:
                        print(f"✓ {name} - SUCCESS")
                    else:
                        print(f"✗ {name} - FAILED")
                    
                    # Wait between tests
                    time.sleep(2.0)
                
                # Summary
                print("\n" + "="*60)
                print("TEST SUMMARY")
                print("="*60)
                for name, success in results.items():
                    status = "✓ PASS" if success else "✗ FAIL"
                    print(f"  {name:20s} {status}")
                print("="*60 + "\n")
        
        finally:
            tester.destroy_node()
            rclpy.shutdown()
    
    else:
        print(f"Unknown command: {command}")


if __name__ == "__main__":
    main()
