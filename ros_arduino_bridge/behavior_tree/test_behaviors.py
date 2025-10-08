#!/usr/bin/env python3
"""
Individual Behavior Node Testing
=================================
Test each behavior node independently before full mission

Usage:
    python3 test_behaviors.py color_sensor
    python3 test_behaviors.py camera
    python3 test_behaviors.py navigation
    python3 test_behaviors.py motors
    python3 test_behaviors.py disease
    python3 test_behaviors.py all

Author: Robotics Dojo 2025
"""

import rclpy
import py_trees
import py_trees_ros
import sys
import time

# Import behavior nodes
from nodes import *


def test_color_sensor():
    """Test color sensor reading"""
    print("\n🧪 TESTING: Color Sensor")
    print("="*60)
    
    rclpy.init()
    node = rclpy.create_node('test_color_sensor')
    
    # Create simple tree
    root = ReadColorSensor("TestColorSensor", timeout=10.0)
    
    # Setup
    root.setup(node=node)
    root.initialise()
    
    # Tick until complete
    status = py_trees.common.Status.RUNNING
    while status == py_trees.common.Status.RUNNING:
        status = root.update()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
    
    # Check result
    blackboard = py_trees.blackboard.Client(name="TestClient")
    blackboard.register_key(key='detected_color', access=py_trees.common.Access.READ)
    detected = blackboard.get('detected_color')
    
    print(f"\n✓ Result: {status}")
    print(f"✓ Detected color: {detected}")
    
    node.destroy_node()
    rclpy.shutdown()


def test_camera():
    """Test camera color detection"""
    print("\n🧪 TESTING: Camera Detection")
    print("="*60)
    
    rclpy.init()
    node = rclpy.create_node('test_camera')
    
    # Setup blackboard with test color
    blackboard = py_trees.blackboard.Client(name="TestSetup")
    blackboard.register_key(key='detected_color', access=py_trees.common.Access.WRITE)
    blackboard.detected_color = 'red'  # Test for red
    
    # Create test tree
    root = MonitorCameraForColor("TestCamera", timeout=20.0)
    root.setup(node=node)
    root.initialise()
    
    # Tick until complete
    status = py_trees.common.Status.RUNNING
    count = 0
    while status == py_trees.common.Status.RUNNING and count < 200:
        status = root.update()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
        count += 1
    
    print(f"\n✓ Result: {status}")
    
    node.destroy_node()
    rclpy.shutdown()


def test_navigation():
    """Test basic navigation"""
    print("\n🧪 TESTING: Navigation")
    print("="*60)
    print("⚠️  WARNING: Robot will move! Clear area around robot.")
    input("Press ENTER to start navigation test...")
    
    rclpy.init()
    
    # Create behavior tree
    root = py_trees.composites.Sequence("NavTest", memory=True)
    
    # Test: Move 1m forward, stop, reverse 1m
    move_forward = MoveRelativeDistance(
        "MoveForward",
        distance_meters=1.0,
        angle_degrees=0,
        linear_speed=0.2
    )
    
    stop = StopRobot("Stop", duration=2.0)
    
    reverse = ReverseDistance(
        "Reverse",
        distance_meters=1.0,
        linear_speed=0.15
    )
    
    root.add_children([move_forward, stop, reverse])
    
    # Create tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(timeout=10.0, node=tree.node)
    
    # Run
    completed = False
    def tick():
        nonlocal completed
        if not completed:
            tree.tick_tock(period_ms=500)
            if tree.root.status != py_trees.common.Status.RUNNING:
                completed = True
                print(f"\n✓ Navigation test: {tree.root.status}")
    
    timer = tree.node.create_timer(0.5, tick)
    
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()


def test_motors():
    """Test motor controls (servos, conveyor)"""
    print("\n🧪 TESTING: Motor Controls")
    print("="*60)
    
    rclpy.init()
    
    # Create test sequence
    root = py_trees.composites.Sequence("MotorTest", memory=True)
    
    # Test camera servo
    camera_servo = ActivateCameraServo("TestCameraServo", target_angle=45)
    wait1 = StopRobot("Wait1", duration=2.0)
    reset_camera = ActivateCameraServo("ResetCamera", target_angle=90)
    
    # Test tipper servo
    tipper_servo = ActivateTipperServo("TestTipper", target_angle=90)
    wait2 = StopRobot("Wait2", duration=2.0)
    reset_tipper = ResetTipperServo("ResetTipper", normal_angle=170)
    
    # Test conveyor (short duration)
    conveyor = ActivateConveyorBelt("TestConveyor", duration=2.0, speed=150)
    
    root.add_children([
        camera_servo, wait1, reset_camera,
        tipper_servo, wait2, reset_tipper,
        conveyor
    ])
    
    # Create tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(timeout=10.0, node=tree.node)
    
    # Run
    completed = False
    def tick():
        nonlocal completed
        if not completed:
            tree.tick_tock(period_ms=500)
            if tree.root.status != py_trees.common.Status.RUNNING:
                completed = True
                print(f"\n✓ Motor test: {tree.root.status}")
    
    timer = tree.node.create_timer(0.5, tick)
    
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()


def test_disease_detection():
    """Test disease detection workflow"""
    print("\n🧪 TESTING: Disease Detection")
    print("="*60)
    
    rclpy.init()
    node = rclpy.create_node('test_disease')
    
    # Create test tree
    root = py_trees.composites.Sequence("DiseaseTest", memory=True)
    
    # Test sequence
    adjust_camera = ActivateCameraServo("AdjustCamera", target_angle=45)
    wait_detection = WaitForDiseaseDetection("WaitDetection", timeout=15.0)
    log_result = LogDiseaseResult("LogResult")
    
    root.add_children([adjust_camera, wait_detection, log_result])
    
    # Setup
    root.setup(node=node)
    root.initialise()
    
    # Tick
    status = py_trees.common.Status.RUNNING
    count = 0
    while status == py_trees.common.Status.RUNNING and count < 200:
        status = root.update()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
        count += 1
    
    print(f"\n✓ Result: {status}")
    
    node.destroy_node()
    rclpy.shutdown()


def print_usage():
    """Print usage instructions"""
    print("\n" + "="*60)
    print("Behavior Node Testing Tool")
    print("="*60)
    print("\nUsage:")
    print("  python3 test_behaviors.py <test_name>")
    print("\nAvailable tests:")
    print("  color_sensor - Test color sensor reading")
    print("  camera       - Test camera color detection")
    print("  navigation   - Test basic movement (⚠️  robot moves!)")
    print("  motors       - Test servo and conveyor controls")
    print("  disease      - Test disease detection workflow")
    print("  all          - Run all tests sequentially")
    print("\nExample:")
    print("  python3 test_behaviors.py color_sensor")
    print("="*60 + "\n")


def main():
    """Main test runner"""
    if len(sys.argv) < 2:
        print_usage()
        return
    
    test_name = sys.argv[1].lower()
    
    tests = {
        'color_sensor': test_color_sensor,
        'camera': test_camera,
        'navigation': test_navigation,
        'motors': test_motors,
        'disease': test_disease_detection,
    }
    
    if test_name == 'all':
        print("\n🧪 RUNNING ALL TESTS")
        print("="*60)
        for name, test_func in tests.items():
            try:
                test_func()
                print(f"✓ {name} test passed\n")
            except Exception as e:
                print(f"❌ {name} test failed: {e}\n")
    elif test_name in tests:
        tests[test_name]()
    else:
        print(f"❌ Unknown test: {test_name}")
        print_usage()


if __name__ == "__main__":
    main()
