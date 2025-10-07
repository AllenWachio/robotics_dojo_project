#!/usr/bin/env python3
"""
Complete Robot Mission - Disease Detection + Cube Pickup and Delivery
=====================================================================

Mission Flow:
PHASE 0 - DISEASE DETECTION:
0.1. Start from origin (0, 0)
0.2. Move left 1.22m, then right 15cm (relative movement)
0.3. Turn 90° left to face the plant display
0.4. Stop and activate camera servo (adjust angle for detection)
0.5. Wait for potato disease detection result
0.6. Return to origin

PHASE 1-9 - CUBE DELIVERY:
1. Move to Point 1 (pickup location)
2. Stop and stabilize (3 seconds)
3. Turn 180° to face opposite direction for proper loading orientation
4. Read cube color with sensor (15 second timeout for manual intervention)
5. Move to Point 2 while Pi camera monitors for matching color
6. Stop when camera detects matching color
7. Execute 180-degree turn at delivery point
8. Reverse 4cm backwards
9. Activate stepper motor for initial offload attempt
10. Verify offload success + retry with tipper servo if cube still detected
11. Mission complete!

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# Import existing behaviors
from robot_navigation_bt import MoveToPosition, make_pose

# Import sensor behaviors (including new disease detection behaviors)
from sensor_behaviors import (
    ReadColorSensor,
    MonitorCameraForColor,
    WaitForColorSensorClear,
    Turn180Degrees,
    Turn90Left,
    ReverseDistance,
    ActivateStepperMotor,
    StopRobot,
    MoveRelativeDistance,
    ActivateCameraServo,
    WaitForDiseaseDetection,
    ActivateTipperServo,
    OffloadWithRetry
)


class MoveAndMonitorCamera(py_trees.behaviour.Behaviour):
    """
    Composite behavior that moves to a position while simultaneously 
    monitoring the Pi camera for the target color.
    
    Stops navigation when camera detects the target color.
    """
    
    def __init__(self, name, target_x, target_y, tolerance=0.3):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.tolerance = tolerance
        
        # Child behaviors (created in setup)
        self.move_behavior = None
        self.camera_monitor = None
        
    def setup(self, **kwargs):
        """Initialize child behaviors"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Create movement behavior
        self.move_behavior = MoveToPosition(
            f"{self.name}_Move",
            self.target_x,
            self.target_y,
            tolerance=self.tolerance
        )
        self.move_behavior.setup(**kwargs)
        
        # Create camera monitor
        self.camera_monitor = MonitorCameraForColor(f"{self.name}_CameraMonitor")
        self.camera_monitor.setup(**kwargs)
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Initialize both child behaviors"""
        self.move_behavior.initialise()
        self.camera_monitor.initialise()
        self.logger.info(f"{self.name}: Moving to ({self.target_x}, {self.target_y}) while monitoring camera...")
    
    def update(self):
        """
        Run both behaviors in parallel - stop when camera detects color
        """
        # Check camera first - highest priority
        camera_status = self.camera_monitor.update()
        if camera_status == py_trees.common.Status.SUCCESS:
            self.logger.info(f"{self.name}: ✓ Camera detected target color! Stopping navigation.")
            # Cancel movement and stop
            return py_trees.common.Status.SUCCESS
        
        # Continue movement
        move_status = self.move_behavior.update()
        
        if move_status == py_trees.common.Status.SUCCESS:
            # Reached destination but camera hasn't detected color yet
            self.logger.warn(f"{self.name}: Reached destination but color not detected yet!")
            # Keep monitoring camera
            return py_trees.common.Status.RUNNING
        elif move_status == py_trees.common.Status.FAILURE:
            self.logger.error(f"{self.name}: Movement failed!")
            return py_trees.common.Status.FAILURE
        else:
            # Keep running
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Clean up child behaviors"""
        if self.move_behavior:
            self.move_behavior.terminate(new_status)
        if self.camera_monitor:
            self.camera_monitor.terminate(new_status)


def create_cube_delivery_tree():
    """
    Creates complete mission behavior tree with disease detection phase first.
    
    Complete Mission Structure:
    ===========================
    
    PHASE 0: DISEASE DETECTION (at origin)
      ├─ MoveRelativeDistance (1.22m left, 0.15m right)
      ├─ Turn90Left (face plant display)
      ├─ StopRobot (stabilize)
      ├─ ActivateCameraServo (adjust camera angle)
      ├─ WaitForDiseaseDetection (get inference result)
      └─ MoveToPosition(0, 0) (return to origin)
    
    PHASE 1-9: CUBE DELIVERY
      ├─ MoveToPosition("Point1", x, y) (pickup location)
      ├─ StopRobot (ensure stopped before reading sensor)
      ├─ ReadColorSensor (detect cube color: red/blue)
      ├─ MoveAndMonitorCamera("Point2", x, y) (move while camera active)
      ├─ StopRobot (stop when color detected)
      ├─ WaitForColorSensorClear (wait for cube offload)
      ├─ Turn180Degrees (rotate in place)
      ├─ ReverseDistance (move back 4cm)
      └─ ActivateStepperMotor (final offload mechanism)
    """
    
    root = py_trees.composites.Sequence("CompleteMission_DiseaseDetection_CubeDelivery", memory=True)
    
    # =============================
    # PHASE 0: DISEASE DETECTION
    # =============================
    print("🔧 Building Complete Mission Tree (Disease Detection + Cube Delivery)...")
    print("\n📋 PHASE 0: POTATO DISEASE DETECTION")
    
    # Step 0.1: Move relative from origin (1.22m left = +Y, 0.15m right = -Y net)
    # Net movement: 1.22m left, then 0.15m right = 1.07m left total
    move_to_plant = MoveRelativeDistance(
        "MoveToPlantDisplay",
        distance_x_m=1.22,    # Forward 1.22m
        distance_y_m=-0.15,   # Then right 0.15m (negative = right in robot frame)
        speed=0.15
    )
    
    # Step 0.2: Turn 90° left to face plant
    turn_to_plant = Turn90Left("TurnToFacePlant", angular_speed=0.5)
    
    # Step 0.3: Stop and stabilize
    stop_for_detection = StopRobot("StopForDetection", duration=1.0)
    
    # Step 0.4: Activate camera servo for optimal viewing angle
    # 45° = 50° (straight) - 5° (angled down slightly for plant detection)
    adjust_camera = ActivateCameraServo("AdjustCameraForPlant", target_angle=45)
    
    # Step 0.5: Wait for disease detection (12 seconds max - then cancel and move on)
    # Wrap in SuccessIsRunning decorator to make timeout non-fatal (converts FAILURE → SUCCESS)
    detect_disease = py_trees.decorators.Timeout(
        name="DiseaseDetectionTimeout",
        child=WaitForDiseaseDetection("DetectPotatoDisease", timeout=12.0),
        duration=12.5  # Slightly longer than behavior timeout
    )
    # Alternative: Use FailureIsSuccess to continue even if detection fails
    detect_disease = py_trees.decorators.FailureIsSuccess(
        name="OptionalDiseaseDetection",
        child=detect_disease
    )
    
    # Step 0.6: Return to origin
    return_to_origin = MoveToPosition(
        "ReturnToOrigin",
        target_x=0.0,
        target_y=0.0,
        tolerance=0.2
    )
    
    print("  ✓ Phase 0: Disease detection sequence configured")
    
    # ===== PHASE 1: Navigate to pickup point and identify cube =====
    print("\n📋 PHASE 1: CUBE PICKUP AND IDENTIFICATION")
    
    # Move to Point 1 (pickup location)
    move_to_point1 = MoveToPosition(
        "MoveToPickup_Point1",
        target_x=2.1,
        target_y=0.0,
        tolerance=0.2  # Stop within 20cm
    )
    
    # Stop and stabilize before reading sensor (longer pause for manual loading)
    stop_at_point1 = StopRobot("StopAtPoint1", duration=3.0)
    
    # Turn 180 degrees for proper loading orientation (face opposite direction)
    turn_for_loading = Turn180Degrees("TurnForLoading", angular_speed=0.5)
    
    # Read color sensor to identify cube (15 second timeout for manual intervention)
    read_color = ReadColorSensor("IdentifyCubeColor")
    
    # ===== PHASE 2: Navigate to delivery point with camera monitoring =====
    print("📋 PHASE 2: CUBE DELIVERY WITH VISION")
    
    # Move to Point 2 while camera monitors for matching color
    # ⚠️ IMPORTANT: Adjust these coordinates to match YOUR delivery location!
    move_and_monitor = MoveAndMonitorCamera(
        "MoveToDelivery_Point2",
        target_x=3.0,  # Example coordinates - UPDATE THESE!
        target_y=1.5,  # Example coordinates - UPDATE THESE!
        tolerance=0.3
    )
    
    # Stop when camera detects matching color
    stop_at_point2 = StopRobot("StopAtPoint2", duration=1.0)
    
    # ===== PHASE 3: Wait for cube offload confirmation =====
    print("📋 PHASE 3: OFFLOAD CONFIRMATION")
    
    # Wait for color sensor to clear (cube removed)
    wait_for_clear = WaitForColorSensorClear("WaitForOffload", timeout=15.0)
    
    # ===== PHASE 4: Execute offload sequence =====
    print("📋 PHASE 4: OFFLOAD MANEUVERS")
    
    # Turn 180 degrees
    turn_around = Turn180Degrees("Turn180", angular_speed=0.5)
    
    # Reverse 4cm
    reverse = ReverseDistance("Reverse4cm", distance_cm=4.0, speed=0.1)
    
    # Activate stepper motor for initial offload attempt
    activate_stepper = ActivateStepperMotor(
        "StepperOffload",
        rpm=-25,        # Negative for offload direction
        distance_mm=400,  # 40cm movement
        flag=0,
        wait_time=5.0   # Wait 5 seconds for stepper to complete
    )
    
    # Check if offload succeeded, retry with tipper servo if needed
    offload_retry = OffloadWithRetry(
        "OffloadVerifyRetry",
        check_timeout=15.0  # Wait 15s max for sensor to clear before retry
    )
    
    # ===== Assemble the complete tree =====
    root.add_children([
        # PHASE 0: Disease Detection
        move_to_plant,         # 0.1. Move to plant display
        turn_to_plant,         # 0.2. Turn to face plant
        stop_for_detection,    # 0.3. Stop and stabilize
        adjust_camera,         # 0.4. Adjust camera servo
        detect_disease,        # 0.5. Wait for detection
        return_to_origin,      # 0.6. Return to origin
        
        # PHASE 1-9: Cube Delivery
        move_to_point1,        # 1. Go to pickup
        stop_at_point1,        # 2. Stop and stabilize
        turn_for_loading,      # 3. Turn 180° for loading orientation
        read_color,            # 4. Identify cube color
        move_and_monitor,      # 5. Go to delivery while monitoring camera
        stop_at_point2,        # 6. Stop at delivery
        turn_around,           # 7. Turn 180° at delivery
        reverse,               # 8. Reverse 4cm
        activate_stepper,      # 9. Initial offload attempt
        offload_retry          # 10. Verify + retry with tipper if needed
    ])
    
    print("\n✅ Complete Mission Tree created successfully!")
    print("\n" + "="*60)
    print("COMPLETE MISSION FLOW:")
    print("="*60)
    print("\n🌱 PHASE 0: DISEASE DETECTION (at origin)")
    print("  0.1 → Move 1.22m forward, 0.15m right (relative)")
    print("  0.2 → Turn 90° left to face plant")
    print("  0.3 ⏸  Stop and stabilize")
    print("  0.4 📷 Adjust camera servo to 45° (50° - 5°)")
    print("  0.5 🔍 Wait for disease detection result (12s timeout, non-fatal)")
    print("  0.6 ← Return to origin (0, 0)")
    print("\n🎯 PHASE 1-9: CUBE DELIVERY")
    print("  1. → Move to Point 1 (pickup)")
    print("  2. ⏸  Stop and stabilize (3 seconds)")
    print("  3. 🔄 Turn 180° for loading orientation")
    print("  4. 📷 Read cube color (15s timeout)")
    print("  5. → Move to Point 2 (camera active)")
    print("  6. ⏸  Stop when camera detects color")
    print("  7. 🔄 Turn 180° at delivery")
    print("  8. ⬅  Reverse 4cm")
    print("  9. ⚙  Activate stepper motor (initial offload)")
    print("  10. 🔍 Check sensor + retry with tipper if needed")
    print("  11. ✅ Mission complete!\n")
    print("="*60 + "\n")
    
    return root


def main():
    """Main function to run the behavior tree"""
    rclpy.init()
    
    # Create behavior tree
    root = create_cube_delivery_tree()
    
    # Wrap in ROS2 behavior tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    try:
        # Setup tree with ROS2 node
        tree.setup(timeout=10.0, node=tree.node)
        
        # Publish initial pose to AMCL (set robot starting position)
        initial_pose_pub = tree.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        # Wait for publisher to initialize
        time.sleep(1.0)
        
        # Set initial pose at origin
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = tree.node.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Publish initial pose multiple times
        for _ in range(5):
            initial_pose_pub.publish(initial_pose)
            time.sleep(0.1)
        
        print("📍 Initial pose published to AMCL")
        print("🚀 Starting Cube Delivery Mission...")
        print("=" * 60)
        
        tree_completed = False
        
        def tick_tree():
            """Tick the behavior tree periodically"""
            nonlocal tree_completed
            
            if tree_completed:
                return
            
            # Execute tree tick
            tree.tick_tock(period_ms=500)
            
            # Check completion
            if tree.root.status == py_trees.common.Status.SUCCESS:
                print("=" * 60)
                print("🎉 MISSION COMPLETED SUCCESSFULLY! 🎉")
                print("=" * 60)
                tree_completed = True
            elif tree.root.status == py_trees.common.Status.FAILURE:
                print("=" * 60)
                print("❌ MISSION FAILED!")
                print("=" * 60)
                tree_completed = True
        
        # Create timer to tick tree at 2Hz
        timer = tree.node.create_timer(0.5, tick_tree)
        
        # Spin ROS2 node
        rclpy.spin(tree.node)
        
    except RuntimeError as e:
        print(f"❌ Setup failed: {e}")
        print("\n⚠️  Make sure the following are running:")
        print("   1. Nav2 navigation stack")
        print("   2. ros_arduino_bridge node")
        print("   3. rpi_camera_package nodes (rpicam_node + color_detection_node)")
        print("   4. SLAM/localization (AMCL)")
    except KeyboardInterrupt:
        print("\n⚠️  Mission interrupted by user")
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
