#!/usr/bin/env python3
"""
IncrementalMove Behavior for py_trees
Moves robot incrementally with obstacle avoidance and recovery.

This behavior coexists with MoveToPosition - use IncrementalMove when you need:
- Progress monitoring at each step
- Automatic obstacle avoidance with LiDAR
- Detailed movement logging
- Recovery from stuck situations

Author: Robotics Dojo 2025
Date: October 6, 2025
"""

import py_trees
import math
import time
from typing import Optional, List, Tuple
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

from incremental_movement_utils import (
    WaypointGenerator,
    PositionTracker,
    MovementLogger,
    ObstacleAvoidance,
    StuckDetector
)


class IncrementalMove(py_trees.behaviour.Behaviour):
    """
    Advanced movement behavior with incremental steps and obstacle avoidance.
    
    Features:
    - Breaks long movements into small steps (default 0.2m)
    - Logs progress at each waypoint
    - Detects stuck situations
    - Uses LiDAR to find alternative routes around obstacles
    - Automatic retry with evasive maneuvers
    - Detailed statistics and reporting
    
    Example Usage:
        # Simple movement (coexists with MoveToPosition)
        move1 = IncrementalMove("GoToKitchen", 3.5, 2.1)
        
        # With custom parameters
        move2 = IncrementalMove("PreciseDocking", 1.0, 0.5,
                                step_size=0.1,      # Smaller steps
                                tolerance=0.05,     # Tighter tolerance
                                max_retries=5)      # More recovery attempts
    """
    
    # State machine states
    STATE_IDLE = 'IDLE'
    STATE_PLANNING = 'PLANNING'
    STATE_MOVING = 'MOVING'
    STATE_CHECKING = 'CHECKING'
    STATE_RECOVERING = 'RECOVERING'
    STATE_GOAL_REACHED = 'GOAL_REACHED'
    STATE_FAILED = 'FAILED'
    
    def __init__(self, name: str, goal_x: float, goal_y: float,
                 step_size: float = 0.2,
                 tolerance: float = 0.15,
                 stuck_timeout: float = 30.0,
                 max_retries: int = 3):
        """
        Initialize IncrementalMove behavior.
        
        Args:
            name: Behavior name for identification
            goal_x, goal_y: Target position in map frame (meters)
            step_size: Distance between waypoints (meters), default 0.2m
            tolerance: Acceptance radius at goal (meters), default 0.15m
            stuck_timeout: Seconds to wait before declaring stuck, default 30s
            max_retries: Maximum recovery attempts, default 3
        """
        super().__init__(name)
        
        # Goal parameters
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.step_size = step_size
        self.tolerance = tolerance
        self.stuck_timeout = stuck_timeout
        self.max_retries = max_retries
        
        # State machine
        self.state = self.STATE_IDLE
        
        # Components (initialized in setup)
        self.position_tracker: Optional[PositionTracker] = None
        self.movement_logger: Optional[MovementLogger] = None
        self.obstacle_avoidance: Optional[ObstacleAvoidance] = None
        self.stuck_detector: Optional[StuckDetector] = None
        
        # ROS2 components
        self.node = None
        self.action_client: Optional[ActionClient] = None
        
        # Waypoint management
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index = 0
        self.start_pose: Optional[Tuple[float, float]] = None
        
        # Action tracking
        self.goal_handle = None
        self.result_future = None
        self.action_sent = False
        self.last_waypoint_pose: Optional[Tuple[float, float]] = None
        
    def setup(self, **kwargs):
        """
        Initialize ROS2 components and utilities.
        Called once when behavior tree is initialized.
        """
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided in setup!")
            return
        
        self.logger.info(f"{self.name}: Initializing IncrementalMove behavior...")
        
        # Initialize position tracker
        self.position_tracker = PositionTracker(self.node)
        
        # Initialize obstacle avoidance
        self.obstacle_avoidance = ObstacleAvoidance(
            self.node,
            safe_distance=0.5,
            min_clearance_angle=30.0
        )
        
        # Initialize action client
        self.action_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Wait for action server
        self.logger.info(f"{self.name}: Waiting for navigation action server...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.logger.warn(f"{self.name}: Navigation action server not available!")
        else:
            self.logger.info(f"{self.name}: Connected to navigation action server")
        
        self.logger.info(f"{self.name}: Setup complete!")
        self.logger.info(f"  Goal: ({self.goal_x:.3f}, {self.goal_y:.3f})")
        self.logger.info(f"  Step size: {self.step_size}m")
        self.logger.info(f"  Tolerance: {self.tolerance}m")
    
    def initialise(self):
        """
        Called when behavior is ticked for the first time or after being reset.
        Sets up the mission and transitions to PLANNING state.
        """
        self.logger.info(f"\n{'='*60}")
        self.logger.info(f"{self.name}: Starting incremental movement mission")
        self.logger.info(f"{'='*60}")
        
        # Reset state
        self.state = self.STATE_IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        self.action_sent = False
        self.goal_handle = None
        self.result_future = None
        
        # Get starting position
        if not self.position_tracker.is_ready():
            self.logger.warn(f"{self.name}: Waiting for position data...")
            return
        
        start_pose_full = self.position_tracker.get_pose()
        self.start_pose = (start_pose_full[0], start_pose_full[1])
        self.last_waypoint_pose = self.start_pose
        
        # Initialize stuck detector
        self.stuck_detector = StuckDetector(
            self.logger,
            min_progress_distance=0.05,
            timeout=self.stuck_timeout,
            max_retries=self.max_retries
        )
        
        # Initialize movement logger
        self.movement_logger = MovementLogger(
            self.logger,
            self.name,
            self.start_pose,
            (self.goal_x, self.goal_y)
        )
        
        # Transition to planning
        self.state = self.STATE_PLANNING
    
    def update(self):
        """
        Main behavior update loop.
        Called every tick of the behavior tree.
        """
        # State machine
        if self.state == self.STATE_IDLE:
            return self._state_idle()
        
        elif self.state == self.STATE_PLANNING:
            return self._state_planning()
        
        elif self.state == self.STATE_MOVING:
            return self._state_moving()
        
        elif self.state == self.STATE_CHECKING:
            return self._state_checking()
        
        elif self.state == self.STATE_RECOVERING:
            return self._state_recovering()
        
        elif self.state == self.STATE_GOAL_REACHED:
            return self._state_goal_reached()
        
        elif self.state == self.STATE_FAILED:
            return self._state_failed()
        
        else:
            self.logger.error(f"{self.name}: Unknown state: {self.state}")
            return py_trees.common.Status.FAILURE
    
    def _state_idle(self):
        """IDLE state - waiting for initialization."""
        # Check if position data is ready
        if not self.position_tracker.is_ready():
            return py_trees.common.Status.RUNNING
        
        # Transition to planning
        self.state = self.STATE_PLANNING
        return py_trees.common.Status.RUNNING
    
    def _state_planning(self):
        """PLANNING state - generate waypoints from current position to goal."""
        self.logger.info(f"{self.name}: Planning waypoints...")
        
        # Get current position
        current_pose = self.position_tracker.get_pose()
        if not current_pose:
            return py_trees.common.Status.RUNNING
        
        current_x, current_y = current_pose[0], current_pose[1]
        
        # Generate waypoints
        self.waypoints = WaypointGenerator.generate(
            current_x, current_y,
            self.goal_x, self.goal_y,
            self.step_size
        )
        
        self.logger.info(f"{self.name}: Generated {len(self.waypoints)} waypoints")
        self.logger.info(f"  From: ({current_x:.3f}, {current_y:.3f})")
        self.logger.info(f"  To:   ({self.goal_x:.3f}, {self.goal_y:.3f})")
        
        # Reset waypoint counter
        self.current_waypoint_index = 0
        
        # Transition to moving
        self.state = self.STATE_MOVING
        return py_trees.common.Status.RUNNING
    
    def _state_moving(self):
        """MOVING state - navigate to next waypoint."""
        # Check if all waypoints completed
        if self.current_waypoint_index >= len(self.waypoints):
            self.logger.info(f"{self.name}: All waypoints completed!")
            self.state = self.STATE_CHECKING
            return py_trees.common.Status.RUNNING
        
        # Get current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_num = self.current_waypoint_index + 1
        
        # Send navigation goal if not already sent
        if not self.action_sent:
            self.logger.info(f"{self.name}: Moving to waypoint {waypoint_num}/{len(self.waypoints)}: ({waypoint[0]:.3f}, {waypoint[1]:.3f})")
            self._send_navigation_goal(waypoint[0], waypoint[1])
            self.action_sent = True
            return py_trees.common.Status.RUNNING
        
        # Check action status
        if self.result_future is None:
            return py_trees.common.Status.RUNNING
        
        if not self.result_future.done():
            # Check for stuck condition
            current_pose = self.position_tracker.get_pose()
            if current_pose:
                if self.stuck_detector.check_progress((current_pose[0], current_pose[1])):
                    self.logger.warn(f"{self.name}: Robot appears stuck!")
                    self._cancel_navigation()
                    self.state = self.STATE_RECOVERING
                    return py_trees.common.Status.RUNNING
            
            return py_trees.common.Status.RUNNING
        
        # Get result
        try:
            result = self.result_future.result()
            self.action_sent = False
            self.result_future = None
            
            # Log progress
            current_pose = self.position_tracker.get_pose()
            if current_pose:
                distance_traveled = WaypointGenerator.distance(
                    self.last_waypoint_pose[0], self.last_waypoint_pose[1],
                    current_pose[0], current_pose[1]
                )
                self.movement_logger.log_waypoint(
                    (current_pose[0], current_pose[1]),
                    distance_traveled
                )
                self.last_waypoint_pose = (current_pose[0], current_pose[1])
            
            # Move to next waypoint
            self.current_waypoint_index += 1
            return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"{self.name}: Navigation failed: {e}")
            self.action_sent = False
            self.state = self.STATE_RECOVERING
            return py_trees.common.Status.RUNNING
    
    def _state_checking(self):
        """CHECKING state - verify goal reached."""
        # Check distance to goal
        distance_to_goal = self.position_tracker.distance_to(self.goal_x, self.goal_y)
        
        self.logger.info(f"{self.name}: Checking goal... distance = {distance_to_goal:.3f}m (tolerance = {self.tolerance}m)")
        
        if distance_to_goal <= self.tolerance:
            self.state = self.STATE_GOAL_REACHED
        else:
            self.logger.warn(f"{self.name}: Goal not reached precisely, replanning...")
            self.state = self.STATE_PLANNING  # Replan to goal
        
        return py_trees.common.Status.RUNNING
    
    def _state_recovering(self):
        """RECOVERING state - attempt obstacle avoidance."""
        self.logger.info(f"{self.name}: Attempting recovery...")
        
        # Check if should retry
        if not self.stuck_detector.should_retry():
            self.logger.error(f"{self.name}: Max retries exceeded!")
            self.state = self.STATE_FAILED
            return py_trees.common.Status.RUNNING
        
        # Increment retry counter
        self.stuck_detector.increment_retry()
        self.movement_logger.log_recovery_attempt(
            self.stuck_detector.get_retry_count(),
            "Obstacle detected, seeking alternative route"
        )
        
        # Find clear direction using LiDAR
        if self.obstacle_avoidance.is_ready():
            clear_angle = self.obstacle_avoidance.find_clear_direction()
            
            if clear_angle is not None:
                self.logger.info(f"{self.name}: Found clear path at {math.degrees(clear_angle):.1f}°")
                
                # Generate evasive waypoint
                current_pose = self.position_tracker.get_pose()
                if current_pose:
                    evasive_waypoint = WaypointGenerator.generate_evasive_waypoint(
                        current_pose[0], current_pose[1],
                        self.goal_x, self.goal_y,
                        clear_angle,
                        evasion_distance=0.5
                    )
                    
                    self.logger.info(f"{self.name}: Moving to evasive position: ({evasive_waypoint[0]:.3f}, {evasive_waypoint[1]:.3f})")
                    
                    # Insert evasive waypoint before current waypoint
                    self.waypoints.insert(self.current_waypoint_index, evasive_waypoint)
                    
                    # Reset stuck detector
                    self.stuck_detector.reset()
                    
                    # Return to moving
                    self.state = self.STATE_MOVING
                    return py_trees.common.Status.RUNNING
        
        # If no clear path found, replan
        self.logger.warn(f"{self.name}: No clear path found, replanning...")
        self.stuck_detector.reset()
        self.state = self.STATE_PLANNING
        return py_trees.common.Status.RUNNING
    
    def _state_goal_reached(self):
        """GOAL_REACHED state - mission complete!"""
        self.logger.info(f"\n{'='*60}")
        self.logger.info(f"{self.name}: ✅ GOAL REACHED!")
        self.logger.info(f"{'='*60}")
        
        # Print summary
        self.movement_logger.print_summary()
        
        return py_trees.common.Status.SUCCESS
    
    def _state_failed(self):
        """FAILED state - mission failed."""
        self.logger.error(f"\n{'='*60}")
        self.logger.error(f"{self.name}: ❌ MISSION FAILED")
        self.logger.error(f"{'='*60}")
        
        # Print summary anyway
        self.movement_logger.print_summary()
        
        return py_trees.common.Status.FAILURE
    
    def _send_navigation_goal(self, x: float, y: float):
        """Send NavigateToPose action to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Handle goal response from action server."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.logger.error(f"{self.name}: Navigation goal rejected!")
            return
        
        self.result_future = self.goal_handle.get_result_async()
    
    def _cancel_navigation(self):
        """Cancel current navigation action."""
        if self.goal_handle:
            self.logger.info(f"{self.name}: Canceling navigation...")
            cancel_future = self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.result_future = None
            self.action_sent = False
    
    def terminate(self, new_status):
        """Called when behavior is terminated."""
        if new_status == py_trees.common.Status.INVALID:
            self._cancel_navigation()
        
        self.logger.info(f"{self.name}: Behavior terminated with status: {new_status}")


# Demo/test code
if __name__ == '__main__':
    print("="*60)
    print("IncrementalMove Behavior - Ready for use!")
    print("="*60)
    print("\nFeatures:")
    print("  ✓ Incremental waypoint generation (0.2m steps)")
    print("  ✓ Real-time position tracking")
    print("  ✓ Obstacle detection with LiDAR")
    print("  ✓ Automatic recovery and retry")
    print("  ✓ Detailed progress logging")
    print("  ✓ Movement statistics")
    print("\nCoexists with MoveToPosition - use as needed!")
    print("="*60)
