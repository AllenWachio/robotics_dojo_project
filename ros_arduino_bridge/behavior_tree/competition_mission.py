#!/usr/bin/env python3
"""
Autonomous Robot Competition Mission
=====================================
Complete behavior tree for maze navigation, disease detection, cargo delivery

Mission Overview:
=================
1. DISEASE DETECTION: Navigate to plant display, detect disease, return to origin
2. CARGO LOADING: Navigate to loading bay, identify cargo color
3. MAZE NAVIGATION: Navigate maze to delivery zone while monitoring camera
4. CARGO DELIVERY: Reverse into delivery bay, offload cargo with conveyor/tipper
5. MISSION COMPLETE: Return to start or standby

Features:
- Dynamic obstacle avoidance (via Nav2)
- Fallback mechanisms for sensor failures
- Manual intervention support
- Comprehensive error handling
- Modular, reusable behavior nodes

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# Import modular behavior nodes
from nodes import (
    # Color Sensor
    ReadColorSensor,
    WaitForColorSensorClear,
    # Camera
    MonitorCameraForColor,
    VerifyColorMatch,
    # Disease Detection
    WaitForDiseaseDetection,
    LogDiseaseResult,
    CheckDiseaseDetectionRequired,
    # Navigation
    MoveToPosition,
    ReverseDistance,
    Turn180Degrees,
    StopRobot,
    MoveRelativeDistance,
    # Motor Control
    ActivateCameraServo,
    ActivateTipperServo,
    ActivateConveyorBelt,
    ResetTipperServo,
)


# ========================================
# COMPETITION WAYPOINTS (MAP COORDINATES)
# ========================================
# Map info: Resolution 0.05m/pixel, Origin (-0.474, -2.22, 0)
# Field size: 2.4m × 2.011m (2400mm × 2011mm)
# 
# IMPORTANT: These are INITIAL ESTIMATES based on field diagram!
# You MUST verify and adjust these using RViz or actual robot testing:
#   1. Launch navigation system
#   2. Use RViz "2D Nav Goal" to click waypoints
#   3. Update these coordinates with actual values
#   4. Test navigation to each point before competition

WAYPOINTS = {
    # Starting position (robot spawn point)
    'start': (0.0, 0.0),
    
    # Disease detection station (plant display area - top section)
    # Located in one of the 480×480mm plant boxes
    'disease_station': (0.5, 1.5),
    
    # Loading bay (middle-left section, 1220×300mm area)
    # Robot backs into this area to load cargo
    'loading_bay': (0.3, 0.8),
    
    # Maze entrance (transition from loading area to delivery zones)
    'maze_entrance': (0.8, 0.0),
    
    # Delivery zones (right section, 1200×1000mm area)
    # Based on colored X markers in navigation path image:
    'green_delivery': (1.5, -1.5),   # Bottom-right (green X in image)
    'red_delivery': (1.5, -0.8),     # Middle-right (red X in image)
    'blue_delivery': (1.5, -0.2),    # Top-right (blue X in image)
}

# HOW TO VERIFY WAYPOINTS:
# ros2 topic echo /amcl_pose  (check robot's current position)
# Use RViz "2D Nav Goal" tool and watch terminal for coordinates


# ========================================
# PHASE 1: DISEASE DETECTION
# ========================================

def create_disease_detection_phase():
    """
    Phase 1: Navigate to disease station, detect disease, return
    
    Structure:
    - Check if disease detection enabled (parameter)
    - If enabled:
        - Move to disease station
        - Adjust camera servo
        - Wait for detection result
        - Log result
        - Return to start
    - If disabled: Skip entirely
    """
    
    # Root selector: skip if disabled
    root = py_trees.composites.Selector("DiseaseDetectionPhase", memory=False)
    
    # Check if enabled
    check_enabled = CheckDiseaseDetectionRequired("CheckDiseaseEnabled")
    
    # Disease detection sequence (only runs if enabled)
    disease_sequence = py_trees.composites.Sequence("DiseaseSequence", memory=True)
    
    # Sub-tasks
    move_to_station = MoveToPosition(
        "MoveToDiseaseStation",
        *WAYPOINTS['disease_station'],
        tolerance=0.3
    )
    
    stop_and_stabilize = StopRobot("StabilizeForDetection", duration=2.0)
    
    adjust_camera = ActivateCameraServo("AdjustCameraForDetection", target_angle=45)
    
    wait_for_detection = WaitForDiseaseDetection("WaitForDetection", timeout=15.0)
    
    log_result = LogDiseaseResult("LogDiseaseResult")
    
    return_to_start = MoveToPosition(
        "ReturnToStart",
        *WAYPOINTS['start'],
        tolerance=0.3
    )
    
    # Assemble disease sequence
    disease_sequence.add_children([
        move_to_station,
        stop_and_stabilize,
        adjust_camera,
        wait_for_detection,
        log_result,
        return_to_start
    ])
    
    # Selector: runs disease sequence only if check_enabled succeeds
    enabled_branch = py_trees.composites.Sequence("EnabledBranch", memory=True)
    enabled_branch.add_children([check_enabled, disease_sequence])
    
    # Fallback that always succeeds (even if disabled)
    root.add_children([
        enabled_branch,
        py_trees.behaviours.Success("DiseaseDetectionSkipped")
    ])
    
    return root


# ========================================
# PHASE 2: CARGO LOADING
# ========================================

def create_cargo_loading_phase():
    """
    Phase 2: Navigate to loading bay, read cargo color
    
    Structure:
    - Navigate to loading bay
    - Reverse into loading position (rear loading)
    - Stop and wait for human to load
    - Read color sensor
    - Exit loading bay (drive forward)
    
    Fallback: Manual color input if sensor fails
    """
    
    root = py_trees.composites.Sequence("CargoLoadingPhase", memory=True)
    
    # Navigate to loading bay
    move_to_loading = MoveToPosition(
        "MoveToLoadingBay",
        *WAYPOINTS['loading_bay'],
        tolerance=0.3
    )
    
    # Reverse into loading position (track robot loads from rear)
    reverse_into_bay = ReverseDistance(
        "ReverseIntoLoadingBay",
        distance_meters=0.5,
        linear_speed=0.15
    )
    
    # Stop and wait for loading
    stop_for_loading = StopRobot("WaitForCargoLoading", duration=5.0)
    
    # Read cargo color (with fallback)
    read_color = create_color_reading_with_fallback()
    
    # Exit loading bay (drive forward)
    exit_loading = MoveRelativeDistance(
        "ExitLoadingBay",
        distance_meters=0.5,
        angle_degrees=0,
        linear_speed=0.2
    )
    
    root.add_children([
        move_to_loading,
        reverse_into_bay,
        stop_for_loading,
        read_color,
        exit_loading
    ])
    
    return root


def create_color_reading_with_fallback():
    """
    Read color sensor with manual fallback
    
    Returns: Always succeeds (either sensor read or manual)
    """
    # Just use ReadColorSensor - it already has timeout fallback built-in
    return ReadColorSensor("ReadCargoColor", timeout=15.0)


# ========================================
# PHASE 3: MAZE NAVIGATION TO DELIVERY
# ========================================

def create_maze_navigation_phase():
    """
    Phase 3: Navigate through maze to delivery zone
    
    Structure:
    - Enter maze
    - Navigate to delivery zone based on detected color
    - Camera monitors for matching color
    - Stop when color detected or reach zone
    
    Fallback: Nav2 handles dynamic obstacle avoidance automatically
    """
    
    root = py_trees.composites.Sequence("MazeNavigationPhase", memory=True)
    
    # Enter maze
    enter_maze = MoveToPosition(
        "EnterMaze",
        *WAYPOINTS['maze_entrance'],
        tolerance=0.3
    )
    
    # Navigate to delivery zone (color-based selection)
    navigate_to_delivery = create_color_based_delivery_selector()
    
    root.add_children([
        enter_maze,
        navigate_to_delivery
    ])
    
    return root


def create_color_based_delivery_selector():
    """
    Select delivery zone based on detected cargo color
    Uses py_trees Selector with condition checks
    
    Structure:
    - Selector (first successful branch wins)
        - Red delivery sequence (check color == red → navigate)
        - Blue delivery sequence (check color == blue → navigate)
        - Green delivery sequence (check color == green → navigate)
        - Default manual delivery (unknown color)
    """
    
    selector = py_trees.composites.Selector("DeliveryZoneSelector", memory=False)
    
    # RED DELIVERY
    red_branch = py_trees.composites.Sequence("RedDeliveryBranch", memory=True)
    red_check = CheckBlackboardColor("CheckRed", expected='red')
    red_nav = MoveToPositionWithCameraMonitor(
        "NavigateToRedZone",
        *WAYPOINTS['red_delivery'],
        tolerance=0.4
    )
    red_branch.add_children([red_check, red_nav])
    
    # BLUE DELIVERY
    blue_branch = py_trees.composites.Sequence("BlueDeliveryBranch", memory=True)
    blue_check = CheckBlackboardColor("CheckBlue", expected='blue')
    blue_nav = MoveToPositionWithCameraMonitor(
        "NavigateToBlueZone",
        *WAYPOINTS['blue_delivery'],
        tolerance=0.4
    )
    blue_branch.add_children([blue_check, blue_nav])
    
    # GREEN DELIVERY (if needed)
    green_branch = py_trees.composites.Sequence("GreenDeliveryBranch", memory=True)
    green_check = CheckBlackboardColor("CheckGreen", expected='green')
    green_nav = MoveToPositionWithCameraMonitor(
        "NavigateToGreenZone",
        *WAYPOINTS['green_delivery'],
        tolerance=0.4
    )
    green_branch.add_children([green_check, green_nav])
    
    # MANUAL FALLBACK (unknown color - go to default zone)
    manual_branch = MoveToPosition(
        "ManualDeliveryFallback",
        *WAYPOINTS['red_delivery'],  # Default to red zone
        tolerance=0.4
    )
    
    selector.add_children([
        red_branch,
        blue_branch,
        green_branch,
        manual_branch  # Always succeeds as fallback
    ])
    
    return selector


class CheckBlackboardColor(py_trees.behaviour.Behaviour):
    """
    Check if detected_color in blackboard matches expected color
    Returns SUCCESS if match, FAILURE if no match
    """
    
    def __init__(self, name, expected):
        super().__init__(name)
        self.expected = expected
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize blackboard"""
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='detected_color',
            access=py_trees.common.Access.READ
        )
    
    def update(self):
        """Check color match"""
        detected = self.blackboard.get('detected_color')
        if detected == self.expected:
            self.logger.info(f"{self.name}: ✓ Color match: '{self.expected}'")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class MoveToPositionWithCameraMonitor(py_trees.behaviour.Behaviour):
    """
    Navigate to position while monitoring camera for color match
    Stops navigation early if camera detects target color
    """
    
    def __init__(self, name, target_x, target_y, tolerance=0.3):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.tolerance = tolerance
        self.move_behavior = None
        self.camera_monitor = None
        
    def setup(self, **kwargs):
        """Initialize child behaviors"""
        self.node = kwargs.get('node')
        
        # Create navigation behavior
        self.move_behavior = MoveToPosition(
            f"{self.name}_Nav",
            self.target_x,
            self.target_y,
            tolerance=self.tolerance
        )
        self.move_behavior.setup(**kwargs)
        
        # Create camera monitor
        self.camera_monitor = MonitorCameraForColor(f"{self.name}_Camera")
        self.camera_monitor.setup(**kwargs)
    
    def initialise(self):
        """Start both behaviors"""
        self.move_behavior.initialise()
        self.camera_monitor.initialise()
    
    def update(self):
        """Run both in parallel - stop when camera detects color"""
        # Check camera first (highest priority)
        camera_status = self.camera_monitor.update()
        if camera_status == py_trees.common.Status.SUCCESS:
            self.logger.info(f"{self.name}: ✓ Camera detected color! Stopping navigation.")
            return py_trees.common.Status.SUCCESS
        
        # Continue navigation
        move_status = self.move_behavior.update()
        return move_status
    
    def terminate(self, new_status):
        """Cleanup"""
        if self.move_behavior:
            self.move_behavior.terminate(new_status)
        if self.camera_monitor:
            self.camera_monitor.terminate(new_status)


# ========================================
# PHASE 4: CARGO DELIVERY
# ========================================

def create_cargo_delivery_phase():
    """
    Phase 4: Reverse into delivery bay, offload cargo
    
    Structure:
    - Verify at correct color zone (using camera)
    - Turn 180° to face away from zone
    - Reverse into delivery bay
    - Offload cargo (with retry mechanism)
    - Exit delivery bay
    
    Fallback: Retry with tipper if first offload fails
    """
    
    root = py_trees.composites.Sequence("CargoDeliveryPhase", memory=True)
    
    # Verify we're at correct delivery zone
    verify_zone = VerifyColorMatch("VerifyDeliveryZone", timeout=5.0)
    
    # Turn around to reverse in
    turn_around = Turn180Degrees("TurnForReverse", angular_speed=0.8, direction='left')
    
    # Reverse into delivery bay
    reverse_into_delivery = ReverseDistance(
        "ReverseIntoDeliveryBay",
        distance_meters=0.4,
        linear_speed=0.12
    )
    
    # Offload with retry mechanism
    offload_cargo = create_offload_with_retry()
    
    # Wait to ensure cargo dropped
    wait_after_offload = StopRobot("WaitAfterOffload", duration=2.0)
    
    # Exit delivery bay
    exit_delivery = MoveRelativeDistance(
        "ExitDeliveryBay",
        distance_meters=0.5,
        angle_degrees=0,
        linear_speed=0.2
    )
    
    root.add_children([
        verify_zone,
        turn_around,
        reverse_into_delivery,
        offload_cargo,
        wait_after_offload,
        exit_delivery
    ])
    
    return root


def create_offload_with_retry():
    """
    Offload cargo with retry mechanism
    
    Structure:
    1. Activate conveyor belt
    2. Check if cargo cleared (color sensor)
    3. If not cleared: retry with tipper servo
    4. Check again
    5. If still not cleared: manual intervention
    """
    
    root = py_trees.composites.Sequence("OffloadWithRetry", memory=True)
    
    # ATTEMPT 1: Conveyor only
    attempt1 = py_trees.composites.Sequence("Attempt1_Conveyor", memory=True)
    conveyor1 = ActivateConveyorBelt("ConveyorAttempt1", duration=4.0, speed=200)
    wait1 = StopRobot("WaitAfterConveyor", duration=1.0)
    attempt1.add_children([conveyor1, wait1])
    
    # Check if cleared
    check1 = WaitForColorSensorClear("CheckAfterAttempt1", timeout=3.0)
    
    # ATTEMPT 2: Conveyor + Tipper (retry if needed)
    retry_branch = py_trees.composites.Sequence("RetryBranch", memory=True)
    
    # Check if retry needed (inverter - runs if check1 failed)
    need_retry = py_trees.composites.Selector("NeedRetry", memory=False)
    need_retry.add_children([
        check1,
        py_trees.behaviours.Success("RetryNeeded")
    ])
    
    # Retry sequence
    attempt2 = py_trees.composites.Sequence("Attempt2_ConveyorWithTipper", memory=True)
    activate_tipper = ActivateTipperServo("ActivateTipper", target_angle=90)
    conveyor2 = ActivateConveyorBelt("ConveyorAttempt2", duration=5.0, speed=255)
    wait2 = StopRobot("WaitAfterTipper", duration=2.0)
    reset_tipper = ResetTipperServo("ResetTipper", normal_angle=170)
    attempt2.add_children([activate_tipper, conveyor2, wait2, reset_tipper])
    
    check2 = WaitForColorSensorClear("CheckAfterAttempt2", timeout=3.0)
    
    retry_branch.add_children([attempt2, check2])
    
    # Final fallback - always succeed (manual intervention)
    final_fallback = py_trees.composites.Selector("FinalOffloadCheck", memory=False)
    final_fallback.add_children([
        retry_branch,
        py_trees.behaviours.Success("ManualOffloadAssumed")
    ])
    
    root.add_children([
        attempt1,
        need_retry,
        final_fallback
    ])
    
    return root


# ========================================
# MAIN COMPETITION TREE
# ========================================

def create_competition_tree():
    """
    Complete competition mission behavior tree
    
    Structure:
    1. Disease Detection Phase
    2. Cargo Loading Phase
    3. Maze Navigation Phase
    4. Cargo Delivery Phase
    5. Mission Complete
    """
    
    root = py_trees.composites.Sequence("CompetitionMission", memory=True)
    
    # Add all phases
    root.add_children([
        create_disease_detection_phase(),
        create_cargo_loading_phase(),
        create_maze_navigation_phase(),
        create_cargo_delivery_phase(),
        py_trees.behaviours.Success("MissionComplete")
    ])
    
    return root


# ========================================
# MAIN EXECUTION
# ========================================

def main():
    """Launch competition mission"""
    rclpy.init()
    
    # Create behavior tree
    root = create_competition_tree()
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    try:
        # Setup tree
        tree.setup(timeout=10.0, node=tree.node)
        
        # Publish initial pose to AMCL
        initial_pose_pub = tree.node.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        time.sleep(1.0)
        
        # Set robot starting position
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = tree.node.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = WAYPOINTS['start'][0]
        initial_pose.pose.pose.position.y = WAYPOINTS['start'][1]
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Publish multiple times to ensure received
        for _ in range(5):
            initial_pose_pub.publish(initial_pose)
            time.sleep(0.1)
        
        print("\n" + "="*70)
        print("🤖 AUTONOMOUS ROBOT COMPETITION MISSION")
        print("="*70)
        print("✓ Behavior tree initialized")
        print("✓ Initial pose published")
        print("✓ Starting mission...")
        print("="*70 + "\n")
        
        # Display tree structure
        py_trees.display.print_ascii_tree(root, show_status=True)
        print("\n")
        
        # Run tree
        tree_completed = False
        start_time = time.time()
        
        def tick_tree():
            nonlocal tree_completed
            if tree_completed:
                return
            
            tree.tick_tock(period_ms=500)
            
            if tree.root.status == py_trees.common.Status.SUCCESS:
                elapsed = time.time() - start_time
                print("\n" + "="*70)
                print("🎉 MISSION COMPLETE!")
                print(f"⏱️  Total time: {elapsed:.1f} seconds ({elapsed/60:.2f} minutes)")
                print("="*70 + "\n")
                tree_completed = True
            elif tree.root.status == py_trees.common.Status.FAILURE:
                print("\n" + "="*70)
                print("❌ MISSION FAILED")
                print("="*70 + "\n")
                tree_completed = True
        
        # Create timer for tree execution
        timer = tree.node.create_timer(0.5, tick_tree)
        
        # Spin
        rclpy.spin(tree.node)
        
    except KeyboardInterrupt:
        print("\n⚠️  Mission interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
