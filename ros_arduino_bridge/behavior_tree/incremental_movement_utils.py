#!/usr/bin/env python3
"""
Utility classes for incremental movement behavior.
Map-agnostic components that work with any coordinate system.

Author: Robotics Dojo 2025
Date: October 6, 2025
"""

import math
import time
from typing import List, Tuple, Optional, Dict
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np


class WaypointGenerator:
    """
    Generates intermediate waypoints between any two positions.
    Map-agnostic - works with any coordinate system using pure math.
    """
    
    @staticmethod
    def generate(start_x: float, start_y: float, 
                 goal_x: float, goal_y: float, 
                 step_size: float = 0.2) -> List[Tuple[float, float]]:
        """
        Generate waypoints from start to goal with specified step size.
        
        Args:
            start_x, start_y: Starting position (meters)
            goal_x, goal_y: Goal position (meters)
            step_size: Distance between waypoints (meters), default 0.2m
            
        Returns:
            List of (x, y) tuples representing waypoints
            
        Example:
            >>> gen = WaypointGenerator()
            >>> waypoints = gen.generate(0.0, 0.0, 1.0, 0.0, step_size=0.2)
            >>> # Returns: [(0.2, 0.0), (0.4, 0.0), (0.6, 0.0), (0.8, 0.0), (1.0, 0.0)]
        """
        # Calculate distance and direction
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Handle case where start == goal
        if distance < 0.001:  # Almost zero
            return [(goal_x, goal_y)]
        
        # Calculate number of steps needed
        num_steps = max(1, int(math.ceil(distance / step_size)))
        
        # Generate waypoints using linear interpolation
        waypoints = []
        for i in range(1, num_steps + 1):
            t = i / num_steps  # Interpolation parameter [0, 1]
            x = start_x + t * dx
            y = start_y + t * dy
            waypoints.append((x, y))
        
        return waypoints
    
    @staticmethod
    def distance(x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    @staticmethod
    def generate_evasive_waypoint(current_x: float, current_y: float,
                                   goal_x: float, goal_y: float,
                                   clear_angle_rad: float,
                                   evasion_distance: float = 0.5) -> Tuple[float, float]:
        """
        Generate an evasive waypoint when obstacle blocks direct path.
        
        Args:
            current_x, current_y: Current position
            goal_x, goal_y: Original goal
            clear_angle_rad: Angle (radians) where path is clear (from LiDAR)
            evasion_distance: How far to move in clear direction (meters)
            
        Returns:
            (x, y) of evasive waypoint
        """
        # Move in the clear direction
        evasive_x = current_x + evasion_distance * math.cos(clear_angle_rad)
        evasive_y = current_y + evasion_distance * math.sin(clear_angle_rad)
        
        return (evasive_x, evasive_y)


class PositionTracker:
    """
    Tracks robot position from AMCL localization.
    Provides distance calculations and pose queries.
    """
    
    def __init__(self, node: Node):
        """
        Initialize position tracker.
        
        Args:
            node: ROS2 node for creating subscription
        """
        self.node = node
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.last_update_time: Optional[float] = None
        
        # Subscribe to AMCL pose
        self.amcl_sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )
        
        self.node.get_logger().info("PositionTracker: Subscribed to /amcl_pose")
    
    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle incoming AMCL pose updates."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.last_update_time = time.time()
    
    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current robot pose.
        
        Returns:
            (x, y, yaw) tuple or None if no pose received yet
        """
        if self.current_x is None:
            return None
        return (self.current_x, self.current_y, self.current_yaw)
    
    def distance_to(self, target_x: float, target_y: float) -> float:
        """
        Calculate distance from current position to target.
        
        Args:
            target_x, target_y: Target position
            
        Returns:
            Distance in meters, or infinity if no pose available
        """
        if self.current_x is None:
            return float('inf')
        
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.sqrt(dx**2 + dy**2)
    
    def is_ready(self) -> bool:
        """Check if position data is available."""
        return self.current_x is not None


class MovementLogger:
    """
    Logs movement progress with detailed statistics.
    Tracks waypoints, distances, times, and calculates performance metrics.
    """
    
    def __init__(self, logger, goal_name: str, start_pose: Tuple[float, float], 
                 goal_pose: Tuple[float, float]):
        """
        Initialize movement logger.
        
        Args:
            logger: ROS2 logger instance
            goal_name: Name of the goal/mission
            start_pose: Starting (x, y)
            goal_pose: Goal (x, y)
        """
        self.logger = logger
        self.goal_name = goal_name
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        
        # History tracking
        self.waypoint_history: List[Tuple[float, float]] = []
        self.timestamps: List[float] = []
        self.distances_traveled: List[float] = []
        self.start_time = time.time()
        self.waypoint_count = 0
        
        # Calculate total mission distance
        self.total_mission_distance = WaypointGenerator.distance(
            start_pose[0], start_pose[1], goal_pose[0], goal_pose[1]
        )
        
        self.logger.info(f"[{goal_name}] Movement tracking started")
        self.logger.info(f"  Start: ({start_pose[0]:.3f}, {start_pose[1]:.3f})")
        self.logger.info(f"  Goal:  ({goal_pose[0]:.3f}, {goal_pose[1]:.3f})")
        self.logger.info(f"  Total distance: {self.total_mission_distance:.3f}m")
    
    def log_waypoint(self, current_pose: Tuple[float, float], 
                     distance_from_last: float):
        """
        Log reaching a waypoint.
        
        Args:
            current_pose: Current (x, y) position
            distance_from_last: Distance traveled since last waypoint
        """
        self.waypoint_count += 1
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Store history
        self.waypoint_history.append(current_pose)
        self.timestamps.append(elapsed)
        self.distances_traveled.append(distance_from_last)
        
        # Calculate progress
        distance_so_far = sum(self.distances_traveled)
        progress_pct = (distance_so_far / self.total_mission_distance) * 100 if self.total_mission_distance > 0 else 0
        
        # Calculate distance remaining to goal
        dist_to_goal = WaypointGenerator.distance(
            current_pose[0], current_pose[1],
            self.goal_pose[0], self.goal_pose[1]
        )
        
        # Log progress
        self.logger.info(f"[{self.goal_name}] ✓ Waypoint {self.waypoint_count}:")
        self.logger.info(f"  Position: ({current_pose[0]:.3f}, {current_pose[1]:.3f})")
        self.logger.info(f"  Progress: {progress_pct:.1f}% ({distance_so_far:.3f}m / {self.total_mission_distance:.3f}m)")
        self.logger.info(f"  Remaining: {dist_to_goal:.3f}m")
        self.logger.info(f"  Step distance: {distance_from_last:.3f}m")
        self.logger.info(f"  Time: {elapsed:.1f}s")
    
    def log_recovery_attempt(self, attempt_num: int, reason: str):
        """Log obstacle recovery attempt."""
        self.logger.warn(f"[{self.goal_name}] 🔄 Recovery Attempt #{attempt_num}: {reason}")
    
    def get_summary(self) -> Dict:
        """
        Get movement statistics summary.
        
        Returns:
            Dictionary with statistics
        """
        total_time = time.time() - self.start_time
        total_distance = sum(self.distances_traveled)
        avg_speed = total_distance / total_time if total_time > 0 else 0
        
        return {
            'total_time': total_time,
            'total_waypoints': self.waypoint_count,
            'total_distance_traveled': total_distance,
            'straight_line_distance': self.total_mission_distance,
            'path_efficiency': (self.total_mission_distance / total_distance * 100) if total_distance > 0 else 0,
            'average_speed': avg_speed,
            'waypoint_history': self.waypoint_history
        }
    
    def print_summary(self):
        """Print final movement summary."""
        summary = self.get_summary()
        
        self.logger.info(f"\n{'='*60}")
        self.logger.info(f"[{self.goal_name}] MOVEMENT SUMMARY")
        self.logger.info(f"{'='*60}")
        self.logger.info(f"Total Time:          {summary['total_time']:.2f}s")
        self.logger.info(f"Waypoints Reached:   {summary['total_waypoints']}")
        self.logger.info(f"Distance Traveled:   {summary['total_distance_traveled']:.3f}m")
        self.logger.info(f"Straight Line Dist:  {summary['straight_line_distance']:.3f}m")
        self.logger.info(f"Path Efficiency:     {summary['path_efficiency']:.1f}%")
        self.logger.info(f"Average Speed:       {summary['average_speed']:.3f}m/s")
        self.logger.info(f"{'='*60}\n")


class ObstacleAvoidance:
    """
    Detects obstacles using LiDAR and finds clear paths.
    Provides obstacle detection and evasive waypoint generation.
    """
    
    def __init__(self, node: Node, safe_distance: float = 0.5, 
                 min_clearance_angle: float = 30.0):
        """
        Initialize obstacle avoidance.
        
        Args:
            node: ROS2 node for creating subscription
            safe_distance: Minimum safe distance from obstacles (meters)
            min_clearance_angle: Minimum angular clearance needed (degrees)
        """
        self.node = node
        self.safe_distance = safe_distance
        self.min_clearance_angle_rad = math.radians(min_clearance_angle)
        
        self.latest_scan: Optional[LaserScan] = None
        self.scan_time: Optional[float] = None
        
        # Subscribe to LiDAR scan
        self.scan_sub = node.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        
        self.node.get_logger().info("ObstacleAvoidance: Subscribed to /scan")
    
    def _scan_callback(self, msg: LaserScan):
        """Handle incoming LiDAR scan data."""
        self.latest_scan = msg
        self.scan_time = time.time()
    
    def is_path_blocked(self, target_angle_rad: float, 
                        check_distance: float = None) -> bool:
        """
        Check if path in given direction is blocked.
        
        Args:
            target_angle_rad: Direction to check (radians, 0 = forward)
            check_distance: Distance to check (meters), defaults to safe_distance
            
        Returns:
            True if blocked, False if clear
        """
        if self.latest_scan is None:
            return False  # No data, assume clear
        
        if check_distance is None:
            check_distance = self.safe_distance
        
        scan = self.latest_scan
        
        # Find scan indices corresponding to target angle
        # Normalize target angle to scan frame
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        
        # Check a cone around target angle
        cone_half_angle = self.min_clearance_angle_rad / 2
        
        for i, range_val in enumerate(scan.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
            
            # Calculate angle for this scan point
            scan_angle = angle_min + i * angle_increment
            
            # Check if within cone
            angle_diff = abs(scan_angle - target_angle_rad)
            if angle_diff <= cone_half_angle:
                if range_val < check_distance:
                    return True  # Obstacle detected!
        
        return False  # Clear!
    
    def find_clear_direction(self) -> Optional[float]:
        """
        Find a clear direction using LiDAR data.
        
        Returns:
            Angle in radians where path is clear, or None if no clear path
        """
        if self.latest_scan is None:
            return None
        
        scan = self.latest_scan
        num_ranges = len(scan.ranges)
        
        # Divide scan into sectors and find clearest
        num_sectors = 12  # Check 12 directions (30° each)
        sector_angles = np.linspace(scan.angle_min, scan.angle_max, num_sectors)
        sector_clearances = []
        
        for sector_angle in sector_angles:
            # Calculate average clearance in this sector
            clearance_sum = 0
            count = 0
            
            for i, range_val in enumerate(scan.ranges):
                if math.isinf(range_val) or math.isnan(range_val):
                    continue
                
                scan_angle = scan.angle_min + i * scan.angle_increment
                angle_diff = abs(scan_angle - sector_angle)
                
                if angle_diff <= self.min_clearance_angle_rad:
                    clearance_sum += min(range_val, 5.0)  # Cap at 5m
                    count += 1
            
            avg_clearance = clearance_sum / count if count > 0 else 0
            sector_clearances.append((sector_angle, avg_clearance))
        
        # Find sector with maximum clearance
        best_sector = max(sector_clearances, key=lambda x: x[1])
        
        # Check if clearance is sufficient
        if best_sector[1] > self.safe_distance:
            return best_sector[0]
        
        return None  # No clear path found
    
    def is_ready(self) -> bool:
        """Check if LiDAR data is available."""
        return self.latest_scan is not None


class StuckDetector:
    """
    Detects if robot is stuck and not making progress.
    Implements retry logic with exponential backoff.
    """
    
    def __init__(self, logger, min_progress_distance: float = 0.05, 
                 timeout: float = 30.0, max_retries: int = 3):
        """
        Initialize stuck detector.
        
        Args:
            logger: ROS2 logger instance
            min_progress_distance: Minimum distance to consider progress (meters)
            timeout: Max time without progress before declaring stuck (seconds)
            max_retries: Maximum recovery attempts before giving up
        """
        self.logger = logger
        self.min_distance = min_progress_distance
        self.timeout = timeout
        self.max_retries = max_retries
        
        self.last_progress_pose: Optional[Tuple[float, float]] = None
        self.last_progress_time: Optional[float] = None
        self.retry_count = 0
        self.stuck_detected = False
    
    def reset(self):
        """Reset detector state."""
        self.last_progress_pose = None
        self.last_progress_time = None
        self.retry_count = 0
        self.stuck_detected = False
    
    def check_progress(self, current_pose: Tuple[float, float]) -> bool:
        """
        Check if robot is making progress.
        
        Args:
            current_pose: Current (x, y) position
            
        Returns:
            True if stuck, False if making progress
        """
        now = time.time()
        
        # First check - initialize
        if self.last_progress_pose is None:
            self.last_progress_pose = current_pose
            self.last_progress_time = now
            return False
        
        # Calculate distance moved since last progress
        dist = WaypointGenerator.distance(
            self.last_progress_pose[0], self.last_progress_pose[1],
            current_pose[0], current_pose[1]
        )
        
        # Made meaningful progress?
        if dist >= self.min_distance:
            self.last_progress_pose = current_pose
            self.last_progress_time = now
            self.stuck_detected = False
            return False
        
        # Check timeout
        time_stuck = now - self.last_progress_time
        if time_stuck > self.timeout:
            if not self.stuck_detected:
                self.logger.warn(f"⚠️ STUCK DETECTED! No progress for {time_stuck:.1f}s")
                self.stuck_detected = True
            return True
        
        return False
    
    def should_retry(self) -> bool:
        """
        Check if should attempt recovery.
        
        Returns:
            True if should retry, False if max retries exceeded
        """
        return self.retry_count < self.max_retries
    
    def increment_retry(self):
        """Increment retry counter."""
        self.retry_count += 1
        self.logger.info(f"Recovery attempt {self.retry_count}/{self.max_retries}")
    
    def get_retry_count(self) -> int:
        """Get current retry count."""
        return self.retry_count


# Test code
if __name__ == '__main__':
    print("="*60)
    print("INCREMENTAL MOVEMENT UTILITIES - DEMO")
    print("="*60)
    
    # Test WaypointGenerator
    print("\n1. Testing WaypointGenerator:")
    print("-" * 40)
    gen = WaypointGenerator()
    
    # Test 1: Short distance
    waypoints = gen.generate(0.0, 0.0, 1.0, 0.0, step_size=0.2)
    print(f"Path from (0,0) to (1,0) with 0.2m steps:")
    for i, (x, y) in enumerate(waypoints, 1):
        print(f"  Waypoint {i}: ({x:.3f}, {y:.3f})")
    
    # Test 2: Diagonal
    waypoints = gen.generate(0.0, 0.0, 1.0, 1.0, step_size=0.3)
    print(f"\nDiagonal path from (0,0) to (1,1) with 0.3m steps:")
    for i, (x, y) in enumerate(waypoints, 1):
        print(f"  Waypoint {i}: ({x:.3f}, {y:.3f})")
    
    # Test 3: Distance calculation
    dist = gen.distance(0, 0, 3, 4)
    print(f"\nDistance from (0,0) to (3,4): {dist:.3f}m (expected: 5.0m)")
    
    # Test 4: Evasive waypoint
    evasive = gen.generate_evasive_waypoint(1.0, 1.0, 3.0, 1.0, 
                                            math.radians(90), 0.5)
    print(f"\nEvasive waypoint from (1,1) toward (3,1), avoiding at 90°:")
    print(f"  Result: ({evasive[0]:.3f}, {evasive[1]:.3f})")
    
    print("\n" + "="*60)
    print("✅ All utility classes ready for use!")
    print("="*60)
