#!/usr/bin/env python3
"""
Navigation Behavior Nodes
=========================
Behaviors for robot movement and navigation
Uses Nav2 for autonomous navigation and cmd_vel for basic movements

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time


class MoveToPosition(py_trees.behaviour.Behaviour):
    """
    Navigate to a specific position using Nav2.
    Supports obstacle avoidance and path replanning.
    
    Args:
        target_x, target_y: Goal position in map frame
        target_yaw: Optional orientation at goal (radians)
        tolerance: Distance tolerance for goal (meters)
    
    Returns: SUCCESS (reached) | FAILURE (navigation failed) | RUNNING
    """
    
    # Shared AMCL pose data
    global_x = 0.0
    global_y = 0.0
    global_yaw = 0.0
    pose_initialized = False
    
    def __init__(self, name, target_x, target_y, target_yaw=0.0, tolerance=0.3):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        self.tolerance = tolerance
        
        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.result_future = None
        self.goal_sent = False
        self.goal_status = None
        
    def setup(self, **kwargs):
        """Initialize Nav2 action client"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Create NavigateToPose action client
        self.action_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Reset for new navigation"""
        self.goal_sent = False
        self.goal_handle = None
        self.result_future = None
        self.goal_status = None
        self.logger.info(f"{self.name}: Navigating to ({self.target_x:.2f}, {self.target_y:.2f})...")
    
    def update(self):
        """Execute navigation"""
        # Send goal if not sent
        if not self.goal_sent:
            # Wait for action server
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                self.logger.info(f"{self.name}: Waiting for Nav2...")
                return py_trees.common.Status.RUNNING
            
            # Create goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._make_pose_stamped(
                self.target_x,
                self.target_y,
                self.target_yaw
            )
            
            # Send goal
            send_future = self.action_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self._goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
        
        # Wait for goal handle
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING
        
        # Check status
        if self.goal_status == 'rejected':
            self.logger.error(f"{self.name}: ❌ Goal rejected by Nav2")
            return py_trees.common.Status.FAILURE
        elif self.goal_status == 'succeeded':
            self.logger.info(f"{self.name}: ✓ Navigation complete!")
            return py_trees.common.Status.SUCCESS
        elif self.goal_status == 'aborted':
            self.logger.error(f"{self.name}: ❌ Navigation aborted")
            return py_trees.common.Status.FAILURE
        elif self.goal_status == 'canceled':
            self.logger.warning(f"{self.name}: ⚠️ Navigation canceled")
            return py_trees.common.Status.FAILURE
        
        # Still navigating
        return py_trees.common.Status.RUNNING
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_status = 'rejected'
        else:
            self.logger.info(f"{self.name}: Goal accepted, navigating...")
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        # Status codes: SUCCEEDED=4, ABORTED=6, CANCELED=5
        if status == 4:
            self.goal_status = 'succeeded'
        elif status == 6:
            self.goal_status = 'aborted'
        elif status == 5:
            self.goal_status = 'canceled'
        else:
            self.goal_status = f'unknown({status})'
    
    def _make_pose_stamped(self, x, y, yaw):
        """Create PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def terminate(self, new_status):
        """Cancel navigation if interrupted"""
        if self.goal_handle and new_status == py_trees.common.Status.INVALID:
            self.goal_handle.cancel_goal_async()


class ReverseDistance(py_trees.behaviour.Behaviour):
    """
    Reverse robot backwards by specified distance.
    Uses open-loop control (no feedback).
    
    Args:
        distance_meters: Distance to reverse (positive value)
        linear_speed: Speed for reversing (m/s)
    
    Returns: Always SUCCESS (after duration)
    """
    
    def __init__(self, name="ReverseDistance", distance_meters=0.5, linear_speed=0.15):
        super().__init__(name)
        self.distance = abs(distance_meters)
        self.speed = abs(linear_speed)
        self.duration = self.distance / self.speed
        
        self.node = None
        self.cmd_vel_pub = None
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize cmd_vel publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info(f"{self.name}: Setup complete (distance={self.distance}m, duration={self.duration:.2f}s)")
    
    def initialise(self):
        """Start reversing"""
        self.start_time = time.time()
        self.logger.info(f"{self.name}: ⬅️ Reversing {self.distance}m...")
    
    def update(self):
        """Execute reverse movement"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            twist = Twist()
            twist.linear.x = -self.speed  # Negative = reverse
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Stop
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.logger.info(f"{self.name}: ✓ Reverse complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Stop robot"""
        twist = Twist()
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class Turn180Degrees(py_trees.behaviour.Behaviour):
    """
    Turn robot 180 degrees in place.
    
    Args:
        angular_speed: Rotation speed (rad/s)
        direction: 'left' (counterclockwise) or 'right' (clockwise)
    
    Returns: Always SUCCESS (after duration)
    """
    
    def __init__(self, name="Turn180Degrees", angular_speed=0.8, direction='left'):
        super().__init__(name)
        self.angular_speed = abs(angular_speed)
        self.direction = direction
        self.target_angle = math.pi  # 180 degrees
        self.duration = self.target_angle / self.angular_speed
        
        self.node = None
        self.cmd_vel_pub = None
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize cmd_vel publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info(f"{self.name}: Setup complete (duration={self.duration:.2f}s)")
    
    def initialise(self):
        """Start turn"""
        self.start_time = time.time()
        self.logger.info(f"{self.name}: 🔄 Turning 180° {self.direction}...")
    
    def update(self):
        """Execute turn"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            twist = Twist()
            if self.direction == 'left':
                twist.angular.z = self.angular_speed  # Positive = left
            else:
                twist.angular.z = -self.angular_speed  # Negative = right
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Stop
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.logger.info(f"{self.name}: ✓ Turn complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Stop rotation"""
        twist = Twist()
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class StopRobot(py_trees.behaviour.Behaviour):
    """
    Stop robot and wait for specified duration.
    Useful for stabilization before sensor readings.
    
    Args:
        duration: Time to wait (seconds)
    
    Returns: Always SUCCESS (after duration)
    """
    
    def __init__(self, name="StopRobot", duration=2.0):
        super().__init__(name)
        self.duration = duration
        self.node = None
        self.cmd_vel_pub = None
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize cmd_vel publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Stop robot"""
        self.start_time = time.time()
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.logger.info(f"{self.name}: 🛑 Stopped, waiting {self.duration}s...")
    
    def update(self):
        """Wait for duration"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            self.logger.info(f"{self.name}: ✓ Wait complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Ensure stopped"""
        twist = Twist()
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class MoveRelativeDistance(py_trees.behaviour.Behaviour):
    """
    Move robot a relative distance and angle from current position.
    Uses open-loop control.
    
    Args:
        distance_meters: Forward distance (positive = forward)
        angle_degrees: Angle to turn after moving
        linear_speed: Movement speed (m/s)
    
    Returns: Always SUCCESS (after completion)
    """
    
    def __init__(self, name="MoveRelativeDistance", distance_meters=1.0, 
                 angle_degrees=0, linear_speed=0.2):
        super().__init__(name)
        self.distance = distance_meters
        self.angle = math.radians(angle_degrees)
        self.linear_speed = abs(linear_speed)
        
        self.move_duration = abs(self.distance) / self.linear_speed
        self.turn_duration = abs(self.angle) / 0.5  # 0.5 rad/s turn speed
        
        self.node = None
        self.cmd_vel_pub = None
        self.start_time = None
        self.phase = 'move'  # 'move' or 'turn'
        
    def setup(self, **kwargs):
        """Initialize publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Start movement"""
        self.start_time = time.time()
        self.phase = 'move'
        self.logger.info(f"{self.name}: Moving {self.distance}m, then turning {math.degrees(self.angle)}°...")
    
    def update(self):
        """Execute movement"""
        elapsed = time.time() - self.start_time
        
        if self.phase == 'move':
            if elapsed < self.move_duration:
                twist = Twist()
                twist.linear.x = self.linear_speed if self.distance > 0 else -self.linear_speed
                self.cmd_vel_pub.publish(twist)
                return py_trees.common.Status.RUNNING
            else:
                # Move complete, start turn
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.phase = 'turn'
                self.start_time = time.time()
                return py_trees.common.Status.RUNNING
        
        elif self.phase == 'turn':
            if self.angle == 0:
                # No turn needed
                self.logger.info(f"{self.name}: ✓ Movement complete")
                return py_trees.common.Status.SUCCESS
            
            elapsed = time.time() - self.start_time
            if elapsed < self.turn_duration:
                twist = Twist()
                twist.angular.z = 0.5 if self.angle > 0 else -0.5
                self.cmd_vel_pub.publish(twist)
                return py_trees.common.Status.RUNNING
            else:
                # Turn complete
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.logger.info(f"{self.name}: ✓ Movement complete")
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Stop robot"""
        twist = Twist()
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)
