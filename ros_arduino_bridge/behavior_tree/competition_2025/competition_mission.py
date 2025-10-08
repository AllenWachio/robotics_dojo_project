#!/usr/bin/env python3
"""
COMPETITION 2025 - AUTONOMOUS MISSION
======================================
Simple, clean implementation inspired by working reference code.

MOVEMENT EXPLANATION:
====================
Robot navigates using MAP FRAME coordinates via Nav2:
  - X-axis: East/Right  →
  - Y-axis: North/Up    ↑
  - Origin: (0, 0) set at robot start position

When you call MoveToPosition("name", x=2.0, y=1.0):
  → Nav2 plans path from CURRENT position to goal (2.0, 1.0)
  → Path considers obstacles from LiDAR costmap
  → Robot follows path using DWB controller
  → Continuously re-plans if obstacles detected

Direction sources:
  - AMCL: Provides current robot pose (x, y, yaw) in map frame
  - Nav2: Plans and executes paths between poses
  - LiDAR: Creates costmap for obstacle avoidance

MISSION PHASES:
===============
1. OPTIONAL: Disease Detection (if enabled)
2. Cargo Loading: Navigate to bay, read RGB sensor, identify color
3. Maze Navigation: Navigate to delivery zone while monitoring camera
4. Cargo Delivery: Reverse in, offload with conveyor/tipper
5. Mission Complete

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# Import behavior nodes
from behaviors import (
    # Navigation
    MoveToPosition,
    StopRobot,
    ReverseDistance,
    Turn180Degrees,
    # Sensors
    ReadColorSensor,
    MonitorCameraForColor,
    VerifyColorMatch,
    # Disease Detection
    WaitForDiseaseDetection,
    LogDiseaseResult,
    CheckDiseaseDetectionRequired,
    # Actuators
    ActivateCameraServo,
    ActivateTipperServo,
    ActivateConveyorBelt,
    ResetTipperServo,
)


# ========================================
# COMPETITION WAYPOINTS (in map frame meters)
# ========================================
# Field dimensions: 2400mm x 2011mm
# Map origin: (-0.474, -2.22, 0) with 0.05m/pixel resolution
# 
# WAYPOINT CALCULATION FROM FIELD IMAGE:
# ----------------------------------------
# Grey X (Start): Left side, ~300mm from left, ~300mm from bottom
#   → Pixel: (300/50, 300/50) = (6, 6) → Map: (-0.474 + 0.3, -2.22 + 0.3) = (-0.17, -1.92)
#
# Green X (Disease Detection): Left side, ~300mm from left, ~1000mm from bottom  
#   → Pixel: (300/50, 1000/50) = (6, 20) → Map: (-0.474 + 0.3, -2.22 + 1.0) = (-0.17, -1.22)
#
# Blue Region (Loading Bay): Top-left, ~518mm from left, ~1700mm from bottom
#   → Pixel: (518/50, 1700/50) = (10.36, 34) → Map: (-0.474 + 0.518, -2.22 + 1.7) = (0.044, -0.52)
#
# Orange Region (Delivery Bay): Right side, multiple zones around ~1800-2100mm from left
#   Red zone: ~1800mm from left, ~1200mm from bottom
#     → Map: (-0.474 + 1.8, -2.22 + 1.2) = (1.326, -1.02)
#   Blue zone: ~1800mm from left, ~800mm from bottom
#     → Map: (-0.474 + 1.8, -2.22 + 0.8) = (1.326, -1.42)
#   Green zone: ~1800mm from left, ~400mm from bottom
#     → Map: (-0.474 + 1.8, -2.22 + 0.4) = (1.326, -1.82)
#
# Maze entrance: Between loading bay and delivery bay, ~1200mm from left, ~1000mm from bottom
#   → Map: (-0.474 + 1.2, -2.22 + 1.0) = (0.726, -1.22)
# ----------------------------------------

WAYPOINTS = {
    'start': (-0.17, -1.92),              # Grey X - Robot starting position
    'disease_station': (-0.17, -1.22),    # Green X - Plant inspection station
    'loading_bay': (0.044, -0.52),        # Blue region - Cargo pickup
    'maze_entrance': (0.726, -1.22),      # Midpoint before delivery zones
    'red_delivery': (1.326, -1.02),       # Orange region - Red cargo zone
    'blue_delivery': (1.326, -1.42),      # Orange region - Blue cargo zone  
    'green_delivery': (1.326, -1.82),     # Orange region - Green cargo zone
}


# ========================================
# TREE CONSTRUCTION
# ========================================

def create_root():
    """Build complete competition behavior tree"""
    root = py_trees.composites.Sequence("MissionRoot", memory=True)
    
    # Phase 1: Disease Detection (optional)
    disease_phase = create_disease_detection_phase()
    
    # Phase 2: Cargo Loading
    loading_phase = create_cargo_loading_phase()
    
    # Phase 3: Maze Navigation
    maze_phase = create_maze_navigation_phase()
    
    # Phase 4: Cargo Delivery
    delivery_phase = create_cargo_delivery_phase()
    
    root.add_children([
        disease_phase,
        loading_phase,
        maze_phase,
        delivery_phase,
    ])
    
    return root


def create_disease_detection_phase():
    """Phase 1: Navigate to plant, detect disease, return"""
    phase = py_trees.composites.Sequence("DiseaseDetectionPhase", memory=True)
    
    check_required = CheckDiseaseDetectionRequired("CheckDiseaseRequired")
    go_to_plant = MoveToPosition(
        "GoToPlant",
        *WAYPOINTS['disease_station'],
        tolerance=0.2
    )
    detect = WaitForDiseaseDetection("DetectDisease", timeout=15.0)
    log_result = LogDiseaseResult("LogDisease")
    return_home = MoveToPosition("ReturnFromPlant", *WAYPOINTS['start'], tolerance=0.2)
    
    phase.add_children([
        check_required,
        go_to_plant,
        detect,
        log_result,
        return_home,
    ])
    
    return phase


def create_cargo_loading_phase():
    """Phase 2: Navigate to loading bay, identify cargo"""
    phase = py_trees.composites.Sequence("CargoLoadingPhase", memory=True)
    
    go_to_bay = MoveToPosition(
        "GoToLoadingBay",
        *WAYPOINTS['loading_bay'],
        tolerance=0.2
    )
    reverse_in = ReverseDistance("ReverseIntoBay", distance=0.5, velocity=0.2)
    read_color = ReadColorSensor("ReadCargoColor", timeout=10.0)
    exit_bay = MoveToPosition("ExitBay", *WAYPOINTS['start'], tolerance=0.2)
    
    phase.add_children([
        go_to_bay,
        reverse_in,
        read_color,
        exit_bay,
    ])
    
    return phase


def create_maze_navigation_phase():
    """Phase 3: Navigate to delivery zone based on cargo color"""
    phase = py_trees.composites.Selector("MazeNavigationPhase", memory=False)
    
    # Parallel: Navigate + Monitor camera
    def create_navigation_to_delivery(color, waypoint_key):
        parallel = py_trees.composites.Parallel(
            f"NavigateTo{color.capitalize()}",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        
        navigate = MoveToPosition(
            f"GoTo{color.capitalize()}Zone",
            *WAYPOINTS[waypoint_key],
            tolerance=0.3
        )
        monitor = MonitorCameraForColor(f"Monitor{color.capitalize()}", target_color=color)
        
        parallel.add_children([navigate, monitor])
        return parallel
    
    # Create branches for each color
    red_branch = create_navigation_to_delivery('red', 'red_delivery')
    blue_branch = create_navigation_to_delivery('blue', 'blue_delivery')
    green_branch = create_navigation_to_delivery('green', 'green_delivery')
    
    phase.add_children([red_branch, blue_branch, green_branch])
    
    return phase


def create_cargo_delivery_phase():
    """Phase 4: Reverse into bay, offload cargo"""
    phase = py_trees.composites.Sequence("CargoDeliveryPhase", memory=True)
    
    verify_color = VerifyColorMatch("VerifyDeliveryZone")
    reverse_into_bay = ReverseDistance("ReverseIntoDelivery", distance=0.6, velocity=0.2)
    stop = StopRobot("StabilizeForOffload", duration=0.5)
    
    # Offload with retry: Try conveyor, then conveyor+tipper
    offload_fallback = py_trees.composites.Selector("OffloadFallback", memory=False)
    
    # First attempt: Conveyor only
    try_conveyor = ActivateConveyorBelt("TryConveyor", duration=3.0, pwm=200)
    
    # Second attempt: Conveyor + Tipper
    tipper_sequence = py_trees.composites.Sequence("ConveyorPlusTipper", memory=True)
    activate_tipper = ActivateTipperServo("TiltRobot", angle=45, duration=2.0)
    conveyor_with_tilt = ActivateConveyorBelt("ConveyorWithTilt", duration=3.0, pwm=220)
    reset_tipper = ResetTipperServo("ResetTilt")
    tipper_sequence.add_children([activate_tipper, conveyor_with_tilt, reset_tipper])
    
    offload_fallback.add_children([try_conveyor, tipper_sequence])
    
    phase.add_children([
        verify_color,
        reverse_into_bay,
        stop,
        offload_fallback,
    ])
    
    return phase


# ========================================
# MAIN
# ========================================

def main():
    rclpy.init()
    
    root = create_root()
    
    # Create ROS2 behavior tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    try:
        tree.setup(timeout=10.0, node=tree.node)
        
        # Publish initial pose to AMCL
        initial_pose_pub = tree.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        time.sleep(1.0)
        
        # Set initial pose to actual starting position from field
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = tree.node.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = WAYPOINTS['start'][0]
        initial_pose.pose.pose.position.y = WAYPOINTS['start'][1]
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0  # Facing forward (0° yaw)
        
        # Publish multiple times to ensure AMCL receives it
        for _ in range(5):
            initial_pose_pub.publish(initial_pose)
            time.sleep(0.1)
        
        print(f"📍 Initial pose set to: {WAYPOINTS['start']}")
        print("\n" + "="*60)
        print("🏁 COMPETITION MISSION STARTING")
        print("="*60 + "\n")
        
        tree_completed = False
        
        def tick_tree():
            nonlocal tree_completed
            
            if tree_completed:
                return
            
            tree.tick_tock(period_ms=500)
            
            if tree.root.status == py_trees.common.Status.SUCCESS:
                print("\n" + "="*60)
                print("✅ MISSION COMPLETED SUCCESSFULLY!")
                print("="*60 + "\n")
                tree_completed = True
            elif tree.root.status == py_trees.common.Status.FAILURE:
                print("\n" + "="*60)
                print("❌ MISSION FAILED")
                print("="*60 + "\n")
                tree_completed = True
        
        timer = tree.node.create_timer(0.5, tick_tree)
        
        rclpy.spin(tree.node)
        
    except KeyboardInterrupt:
        print("\n⏸️  Mission interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
