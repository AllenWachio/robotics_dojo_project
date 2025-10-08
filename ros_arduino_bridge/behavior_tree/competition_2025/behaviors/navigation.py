#!/usr/bin/env python3
"""
Navigation Behaviors - Competition 2025
========================================
Enhanced navigation behaviors using Nav2 with proper AMCL pose tracking.
Inspired by working reference implementation.

Key Features:
- Shared AMCL pose subscription (singleton pattern)
- Nav2 action client for autonomous navigation
- Goal tolerance configuration
- Real-time distance monitoring
- Proper async callback handling

Movement Directions:
====================
The robot moves in the MAP FRAME coordinate system:
  - X-axis: Points to the RIGHT (East)
  - Y-axis: Points UP (North)
  - Origin: Set by map.yaml origin parameter

When you command MoveToPosition(x=2.0, y=1.0):
  - Robot moves 2.0m in X direction (right/east)
  - Robot moves 1.0m in Y direction (up/north)
  - From the CURRENT position in map frame

Navigation is handled by Nav2 stack which:
  1. Plans path from current pose to goal pose
  2. Considers obstacles from LiDAR costmap
  3. Executes path using DWB controller
  4. Continuously re-plans if obstacles detected

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import math
import time


class MoveToPosition(py_trees.behaviour.Behaviour):
    """
    Navigate to absolute position in map frame using Nav2.
    
    This behavior uses the Nav2 navigation stack which:
    - Plans optimal path considering obstacles
    - Dynamically avoids obstacles using LiDAR
    - Re-plans if path becomes blocked
    - Uses AMCL for localization
    
    Direction/Movement Explanation:
    ================================
    The robot navigates in MAP FRAME coordinates:
    
        ↑ Y (North)
        |
        |
        └────→ X (East)
    
    Example: MoveToPosition("GoToGoal", 2.0, 1.5)
    - Navigates to map coordinates (2.0, 1.5)
    - X=2.0: 2 meters East from map origin
    - Y=1.5: 1.5 meters North from map origin
    
    The path taken depends on:
    - Current robot position (from AMCL)
    - Obstacles in costmap (from LiDAR)
    - Nav2 planner algorithm (usually Dijkstra or A*)
    
    Args:
        name: Behavior node name
        target_x: Goal X coordinate in map frame (meters)
        target_y: Goal Y coordinate in map frame (meters)
        tolerance: Distance tolerance for reaching goal (meters, default 0.2)
    
    Returns:
        - RUNNING: Navigation in progress
        - SUCCESS: Reached goal within tolerance
        - FAILURE: Navigation rejected or aborted
    
    Blackboard Outputs:
        - current_position: (x, y, yaw) tuple updated continuously
    """
    
    # ============================================
    # SHARED CLASS VARIABLES (Singleton pattern)
    # ============================================
    # All MoveToPosition instances share ONE AMCL subscription
    # This prevents multiple subscriptions to same topic
    global_x = 0.0
    global_y = 0.0
    global_yaw = 0.0
    pose_initialized = False
    pose_sub = None  # Single shared subscription
    
    def __init__(self, name, target_x, target_y, tolerance=0.2):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.tolerance = tolerance
        
        # Action client for Nav2
        self.action_client = None
        self.goal_handle = None
        self.result_future = None
        self.goal_sent = False
        self.goal_status = None
        
        # Parameter client for tolerance setting
        self.param_client = None
        self.tolerance_set = False
        
        # Completion tracking
        self.completed = False
    
    def setup(self, **kwargs):
        """
        Setup behavior with ROS node.
        Creates Nav2 action client and shared AMCL subscription.
        """
        node = kwargs.get('node')
        if not node:
            self.logger.error(f"{self.name}: No ROS node provided!")
            return
        
        self.node = node
        
        # Create NavigateToPose action client
        self.action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose'
        )
        
        # Create parameter client for goal tolerance
        self.param_client = node.create_client(
            SetParameters, '/controller_server/set_parameters'
        )
        
        # Create SHARED AMCL pose subscription (singleton)
        # Only create if it doesn't exist yet
        if MoveToPosition.pose_sub is None:
            MoveToPosition.pose_sub = node.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                MoveToPosition.shared_pose_callback,
                10
            )
            self.logger.info("📍 AMCL pose subscription created (shared)")
        
        self.logger.info(f"{self.name}: Setup completed")
    
    @classmethod
    def shared_pose_callback(cls, msg):
        """
        Shared callback for AMCL pose updates.
        Updates class variables accessible to all instances.
        
        AMCL provides:
        - Position: Robot location in map frame
        - Orientation: Robot heading as quaternion
        - Covariance: Localization uncertainty
        """
        # Extract position (already in map frame)
        cls.global_x = msg.pose.pose.position.x
        cls.global_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw angle
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        cls.global_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        cls.pose_initialized = True
    
    def initialise(self):
        """Reset state for new execution"""
        self.goal_sent = False
        self.goal_handle = None
        self.result_future = None
        self.goal_status = None
        self.completed = False
        self.logger.info(
            f"{self.name}: Starting navigation to ({self.target_x:.2f}, {self.target_y:.2f})"
        )
    
    def update(self):
        """
        Execute navigation behavior.
        
        State machine:
        1. Wait for AMCL pose
        2. Set goal tolerance parameter
        3. Send navigation goal to Nav2
        4. Monitor progress and distance
        5. Return SUCCESS when goal reached
        """
        # Quick exit if already completed
        if self.completed:
            return py_trees.common.Status.SUCCESS
        
        # Step 1: Wait for AMCL localization
        if not MoveToPosition.pose_initialized:
            self.logger.info(f"{self.name}: ⏳ Waiting for AMCL localization...")
            return py_trees.common.Status.RUNNING
        
        # Step 2: Configure goal tolerance
        if not self.tolerance_set:
            if not self.param_client.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"{self.name}: ⏳ Waiting for parameter service...")
                return py_trees.common.Status.RUNNING
            
            # Set xy_goal_tolerance for Nav2 controller
            request = SetParameters.Request()
            
            param = Parameter()
            param.name = 'general_goal_checker.xy_goal_tolerance'
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=self.tolerance
            )
            request.parameters = [param]
            
            future = self.param_client.call_async(request)
            future.add_done_callback(self._param_callback)
            self.tolerance_set = True  # Don't retry
            return py_trees.common.Status.RUNNING
        
        # Step 3: Send navigation goal
        if not self.goal_sent:
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                self.logger.info(f"{self.name}: ⏳ Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._make_pose(
                self.target_x, self.target_y, yaw=0.0
            )
            
            # Send goal asynchronously
            self.logger.info(f"{self.name}: 🎯 Sending goal to Nav2...")
            send_future = self.action_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self._goal_response_callback)
            
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
        
        # Step 4: Wait for goal acceptance
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING
        
        # Check for rejection
        if self.goal_status == 'rejected':
            self.logger.error(f"{self.name}: ❌ Goal rejected by Nav2!")
            return py_trees.common.Status.FAILURE
        
        # Step 5: Monitor navigation progress
        current_x = MoveToPosition.global_x
        current_y = MoveToPosition.global_y
        
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        self.logger.info(
            f"{self.name}: 🚗 Position ({current_x:.2f}, {current_y:.2f}) "
            f"→ Goal ({self.target_x:.2f}, {self.target_y:.2f}), "
            f"Distance: {distance:.2f}m"
        )
        
        # Update blackboard with current position
        blackboard = self.blackboard_client()
        blackboard.register_key('current_position', write=True)
        blackboard.current_position = (current_x, current_y, MoveToPosition.global_yaw)
        
        # Check completion
        if self.goal_status == 'succeeded':
            self.logger.info(
                f"{self.name}: ✅ Navigation complete! Final distance: {distance:.2f}m"
            )
            self.completed = True
            return py_trees.common.Status.SUCCESS
        
        elif self.goal_status in ['aborted', 'canceled']:
            self.logger.error(f"{self.name}: ❌ Navigation {self.goal_status}!")
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING
    
    def _param_callback(self, future):
        """Handle parameter setting response"""
        try:
            response = future.result()
            if response and all(r.successful for r in response.results):
                self.logger.info(f"{self.name}: ✓ Goal tolerance set to {self.tolerance}m")
            else:
                self.logger.warning(f"{self.name}: ⚠ Parameter setting incomplete")
        except Exception as e:
            self.logger.warning(f"{self.name}: ⚠ Parameter error: {e}")
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.logger.error(f"{self.name}: ❌ Goal rejected!")
            self.goal_status = 'rejected'
        else:
            self.logger.info(f"{self.name}: ✓ Goal accepted, navigating...")
            # Request result asynchronously
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        # Action status codes from action_msgs/msg/GoalStatus
        if status == 4:  # SUCCEEDED
            self.goal_status = 'succeeded'
        elif status == 6:  # ABORTED
            self.goal_status = 'aborted'
        elif status == 5:  # CANCELED
            self.goal_status = 'canceled'
        else:
            self.goal_status = f'unknown({status})'
    
    def _make_pose(self, x, y, yaw=0.0, frame="map"):
        """Create PoseStamped message for Nav2 goal"""
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp.sec = 0  # Use current time
        pose.header.stamp.nanosec = 0
        
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose


class StopRobot(py_trees.behaviour.Behaviour):
    """
    Immediately stop robot motion by publishing zero velocity.
    Useful for emergency stops or stabilization before actions.
    
    Direction: N/A (stops all motion)
    """
    
    def __init__(self, name="StopRobot", duration=0.5):
        super().__init__(name)
        self.duration = duration
        self.cmd_vel_pub = None
        self.start_time = None
    
    def setup(self, **kwargs):
        node = kwargs.get('node')
        if node:
            self.node = node
            self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    def initialise(self):
        self.start_time = time.time()
        self.logger.info(f"{self.name}: 🛑 Stopping robot...")
    
    def update(self):
        # Publish zero velocity
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Wait for duration
        if time.time() - self.start_time >= self.duration:
            self.logger.info(f"{self.name}: ✓ Robot stopped")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class ReverseDistance(py_trees.behaviour.Behaviour):
    """
    Drive robot backwards for specified distance using open-loop control.
    
    Direction: BACKWARDS (negative X in robot frame)
    - Robot moves in reverse (away from front)
    - Uses constant velocity
    - No obstacle detection (open-loop)
    
    Use for:
    - Backing into loading bays
    - Exiting tight spaces
    - Short precise movements
    
    Args:
        distance: How far to reverse in meters (positive value)
        velocity: Reverse speed in m/s (positive value)
    """
    
    def __init__(self, name, distance=0.5, velocity=0.2):
        super().__init__(name)
        self.distance = distance
        self.velocity = abs(velocity)  # Ensure positive
        self.cmd_vel_pub = None
        self.start_time = None
        self.duration = distance / velocity
    
    def setup(self, **kwargs):
        node = kwargs.get('node')
        if node:
            self.node = node
            self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    def initialise(self):
        self.start_time = time.time()
        self.logger.info(
            f"{self.name}: ⏪ Reversing {self.distance:.2f}m "
            f"at {self.velocity:.2f}m/s ({self.duration:.1f}s)"
        )
    
    def update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            # Publish reverse velocity
            cmd = Twist()
            cmd.linear.x = -self.velocity  # Negative = backwards
            self.cmd_vel_pub.publish(cmd)
            return py_trees.common.Status.RUNNING
        else:
            # Stop and complete
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.logger.info(f"{self.name}: ✓ Reverse complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Ensure robot stops when behavior ends"""
        if self.cmd_vel_pub:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)


class Turn180Degrees(py_trees.behaviour.Behaviour):
    """
    Rotate robot 180 degrees in place.
    
    Direction: ROTATIONAL (around Z-axis)
    - Positive angular_z = counter-clockwise
    - Negative angular_z = clockwise
    - Default: clockwise 180° turn
    
    Use for:
    - Turning around to exit loading bay
    - Changing direction in maze
    """
    
    def __init__(self, name="Turn180", angular_velocity=0.5):
        super().__init__(name)
        self.angular_velocity = angular_velocity
        self.cmd_vel_pub = None
        self.start_time = None
        self.duration = math.pi / angular_velocity  # 180° = π radians
    
    def setup(self, **kwargs):
        node = kwargs.get('node')
        if node:
            self.node = node
            self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    def initialise(self):
        self.start_time = time.time()
        self.logger.info(f"{self.name}: 🔄 Turning 180 degrees...")
    
    def update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            cmd = Twist()
            cmd.angular.z = -self.angular_velocity  # Clockwise
            self.cmd_vel_pub.publish(cmd)
            return py_trees.common.Status.RUNNING
        else:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.logger.info(f"{self.name}: ✓ Turn complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        if self.cmd_vel_pub:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
