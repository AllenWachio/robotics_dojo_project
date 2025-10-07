#!/usr/bin/env python3
"""
Quick Test Script for Complete Mission Components
Tests each behavior individually before running the complete mission
Includes tests for BOTH disease detection and cube delivery phases
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, ColorRGBA, Int32
from geometry_msgs.msg import Twist
import time
import sys


class MissionTester(Node):
    def __init__(self):
        super().__init__('mission_tester')
        self.get_logger().info('🧪 Complete Mission Component Tester started')
        
        # Publishers
        self.color_led_pub = self.create_publisher(Bool, '/color_sensor/led', 10)
        self.stepper_pub = self.create_publisher(String, '/stepper/command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int32, '/camera_servo/command', 10)
        
        # Subscribers
        self.color_sub = self.create_subscription(
            ColorRGBA, '/color_sensor/rgb', self.color_callback, 10)
        self.color_detect_sub = self.create_subscription(
            String, '/color_detection/detected', self.camera_callback, 10)
        self.servo_feedback_sub = self.create_subscription(
            Int32, '/camera_servo/angle', self.servo_callback, 10)
        self.disease_sub = self.create_subscription(
            String, '/inference_result', self.disease_callback, 10)
        
        self.latest_color = None
        self.detected_colors = []
        self.servo_angle = None
        self.disease_result = None
        
        time.sleep(1.0)  # Wait for publishers to initialize
    
    def color_callback(self, msg):
        self.latest_color = (msg.r, msg.g, msg.b)
    
    def camera_callback(self, msg):
        if msg.data not in self.detected_colors:
            self.detected_colors.append(msg.data)
            self.get_logger().info(f'📷 Camera detected: {msg.data.upper()}')
    
    def servo_callback(self, msg):
        self.servo_angle = msg.data
    
    def disease_callback(self, msg):
        self.disease_result = msg.data
        self.get_logger().info(f'🔍 Disease Detection: {msg.data}')
    
    def test_color_sensor(self):
        """Test 1: Color Sensor"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 1: Color Sensor')
        self.get_logger().info('='*60)
        
        # Turn on LED
        led_msg = Bool()
        led_msg.data = True
        self.color_led_pub.publish(led_msg)
        self.get_logger().info('✅ LED turned ON')
        
        # Wait for reading
        self.get_logger().info('⏳ Waiting 2 seconds for color reading...')
        time.sleep(2.0)
        
        if self.latest_color:
            r, g, b = self.latest_color
            self.get_logger().info(f'📊 RGB: R={r:.3f} G={g:.3f} B={b:.3f}')
            
            # Classify
            if r > g and r > b and r > 0.3:
                self.get_logger().info('🔴 Detected: RED')
            elif b > r and b > g and b > 0.3:
                self.get_logger().info('🔵 Detected: BLUE')
            else:
                self.get_logger().warn('⚪ Detected: UNKNOWN/NO COLOR')
        else:
            self.get_logger().error('❌ No color data received!')
        
        # Turn off LED
        led_msg.data = False
        self.color_led_pub.publish(led_msg)
        self.get_logger().info('✅ LED turned OFF')
    
    def test_camera_detection(self):
        """Test 2: Camera Color Detection"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 2: Camera Color Detection')
        self.get_logger().info('='*60)
        self.get_logger().info('⏳ Monitoring camera for 5 seconds...')
        self.get_logger().info('   Place RED or BLUE object in front of camera!')
        
        self.detected_colors = []
        time.sleep(5.0)
        
        if self.detected_colors:
            self.get_logger().info(f'✅ Camera detected: {", ".join(self.detected_colors)}')
        else:
            self.get_logger().warn('⚠️  No colors detected by camera')
    
    def test_turn(self):
        """Test 3: 180-degree turn"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 3: 180-degree Turn')
        self.get_logger().info('='*60)
        self.get_logger().info('⚠️  Robot will rotate! Make sure area is clear!')
        self.get_logger().info('⏳ Starting turn in 3 seconds...')
        time.sleep(3.0)
        
        # Calculate turn duration (180 degrees at 0.5 rad/s)
        import math
        angular_speed = 0.5
        duration = math.pi / angular_speed
        
        self.get_logger().info(f'🔄 Turning 180° at {angular_speed} rad/s for {duration:.1f}s')
        
        # Turn
        start = time.time()
        while (time.time() - start) < duration:
            twist = Twist()
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('✅ Turn complete')
    
    def test_reverse(self):
        """Test 4: Reverse 4cm"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 4: Reverse 4cm')
        self.get_logger().info('='*60)
        self.get_logger().info('⚠️  Robot will reverse! Make sure area is clear!')
        self.get_logger().info('⏳ Starting reverse in 3 seconds...')
        time.sleep(3.0)
        
        distance_cm = 4.0
        speed = 0.1  # m/s
        duration = (distance_cm / 100.0) / speed
        
        self.get_logger().info(f'⬅  Reversing {distance_cm}cm at {speed} m/s for {duration:.2f}s')
        
        # Reverse
        start = time.time()
        while (time.time() - start) < duration:
            twist = Twist()
            twist.linear.x = -speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('✅ Reverse complete')
    
    def test_stepper(self):
        """Test 5: Stepper Motor"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 5: Stepper Motor')
        self.get_logger().info('='*60)
        
        command = String()
        command.data = '-25:400:0'
        
        self.get_logger().info(f'⚙  Sending stepper command: {command.data}')
        self.stepper_pub.publish(command)
        
        self.get_logger().info('⏳ Waiting 5 seconds for stepper to complete...')
        time.sleep(5.0)
        
        self.get_logger().info('✅ Stepper command sent (check /stepper/debug for status)')


    def test_relative_movement(self):
        """Test 6: Relative Movement (Disease Detection Phase)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 6: Relative Movement')
        self.get_logger().info('='*60)
        
        input('⚠️  Robot will move 0.5m forward, then 0.2m right. Press ENTER to start...')
        
        # Phase 1: Forward movement
        self.get_logger().info('Moving 0.5m forward at 0.15 m/s...')
        twist = Twist()
        twist.linear.x = 0.15
        duration = 0.5 / 0.15  # ~3.3 seconds
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('✓ Forward movement complete')
        time.sleep(0.5)
        
        # Phase 2: Right movement
        self.get_logger().info('Moving 0.2m right at 0.15 m/s...')
        twist = Twist()
        twist.linear.y = -0.15  # Negative = right
        duration = 0.2 / 0.15  # ~1.3 seconds
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('✓ Right movement complete')
        
        print('\n✅ Test 6: Relative Movement Test Complete\n')


    def test_turn_90_left(self):
        """Test 7: 90° Left Turn (Disease Detection Phase)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 7: 90° Left Turn')
        self.get_logger().info('='*60)
        
        input('⚠️  Robot will turn 90° left. Press ENTER to start...')
        
        import math
        angular_speed = 0.5  # rad/s
        angle = math.pi / 2.0  # 90 degrees
        duration = angle / angular_speed  # ~3.14 seconds
        
        self.get_logger().info(f'Turning left at {angular_speed} rad/s for {duration:.2f}s...')
        
        twist = Twist()
        twist.angular.z = angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('✓ 90° turn complete')
        print('\n✅ Test 7: 90° Turn Test Complete\n')


    def test_camera_servo(self):
        """Test 8: Camera Servo Control (Disease Detection Phase)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 8: Camera Servo Control')
        self.get_logger().info('='*60)
        
        self.get_logger().info('Testing camera servo angles...')
        
        # Test angle 1: 45° (disease detection position)
        self.get_logger().info('Moving servo to 45°...')
        msg = Int32()
        msg.data = 45
        self.servo_pub.publish(msg)
        
        time.sleep(2.0)
        if self.servo_angle is not None:
            self.get_logger().info(f'✓ Servo at {self.servo_angle}°')
        else:
            self.get_logger().warning('⚠️  No servo feedback received')
        
        time.sleep(1.0)
        
        # Test angle 2: 0° (looking down)
        self.get_logger().info('Moving servo to 0°...')
        msg.data = 0
        self.servo_pub.publish(msg)
        
        time.sleep(2.0)
        if self.servo_angle is not None:
            self.get_logger().info(f'✓ Servo at {self.servo_angle}°')
        
        time.sleep(1.0)
        
        # Return to 45°
        self.get_logger().info('Returning servo to 45°...')
        msg.data = 45
        self.servo_pub.publish(msg)
        time.sleep(2.0)
        
        print('\n✅ Test 8: Camera Servo Test Complete\n')


    def test_disease_detection(self):
        """Test 9: Disease Detection Wait (Disease Detection Phase)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 9: Disease Detection')
        self.get_logger().info('='*60)
        
        self.get_logger().info('Waiting for disease detection result from /inference_result...')
        self.get_logger().info('Make sure potato_disease_detection_node is running!')
        self.get_logger().info('ros2 run rdj2025_potato_disease_detection potato_disease_detection_node')
        
        self.disease_result = None
        timeout = 10.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.disease_result is not None:
                self.get_logger().info(f'✓ Detection received: {self.disease_result}')
                break
            rclpy.spin_once(self, timeout_sec=0.5)
        
        if self.disease_result is None:
            self.get_logger().warning('⚠️  No detection result received within timeout')
            self.get_logger().info('Check that:')
            self.get_logger().info('  1. potato_disease_detection_node is running')
            self.get_logger().info('  2. Camera is publishing to /image')
            self.get_logger().info('  3. Plant leaf is visible to camera')
        else:
            self.get_logger().info(f'✅ Disease Detection: {self.disease_result}')
        
        print('\n✅ Test 9: Disease Detection Test Complete\n')


def print_usage():
    print('\n' + '='*60)
    print('Complete Mission Component Tester')
    print('='*60)
    print('\nUsage: python3 test_mission_components.py <test_name>')
    print('\nCUBE DELIVERY TESTS:')
    print('  color      - Test color sensor (TCS34725)')
    print('  camera     - Test Pi camera color detection')
    print('  turn       - Test 180° turn (⚠️  robot moves!)')
    print('  reverse    - Test reverse movement (⚠️  robot moves!)')
    print('  stepper    - Test stepper motor')
    print('\nDISEASE DETECTION TESTS:')
    print('  relative   - Test relative movement (⚠️  robot moves!)')
    print('  turn90     - Test 90° left turn (⚠️  robot rotates!)')
    print('  servo      - Test camera servo control')
    print('  disease    - Test disease detection waiting')
    print('\nCOMPREHENSIVE TESTS:')
    print('  all        - Test all cube delivery components (⚠️  robot moves!)')
    print('\n' + '='*60 + '\n')


def main():
    if len(sys.argv) < 2:
        print_usage()
    rclpy.init()
    
    tester = MissionTester()
    
    print('\n' + '='*60)
    print('🧪 CUBE DELIVERY MISSION - COMPONENT TESTER')
    print('='*60)
    print('\nThis script will test each component of the mission:')
    print('  1. Color Sensor (TCS34725)')
    print('  2. Camera Color Detection')
    print('  3. 180-degree Turn')
    print('  4. Reverse 4cm')
    print('  5. Stepper Motor')
    print('\n⚠️  WARNING: Tests 3-5 will move the robot!')
    print('   Make sure the robot has clearance and is on the ground.\n')
    
    if len(sys.argv) > 1:
        test_name = sys.argv[1].lower()
    else:
        print('Usage: python3 test_mission_components.py [test_name]')
        print('  test_name: color | camera | turn | reverse | stepper | all')
        print('\nExample: python3 test_mission_components.py color')
        print('         python3 test_mission_components.py all\n')
        return
    
    try:
        if test_name == 'color':
            tester.test_color_sensor()
        elif test_name == 'camera':
            tester.test_camera_detection()
        elif test_name == 'turn':
            tester.test_turn()
        elif test_name == 'reverse':
            tester.test_reverse()
        elif test_name == 'stepper':
            tester.test_stepper()
        elif test_name == 'relative':
            tester.test_relative_movement()
        elif test_name == 'turn90':
            tester.test_turn_90_left()
        elif test_name == 'servo':
            tester.test_camera_servo()
        elif test_name == 'disease':
            tester.test_disease_detection()
        elif test_name == 'all':
            tester.test_color_sensor()
            input('\nPress ENTER to continue to camera test...')
            tester.test_camera_detection()
            input('\n⚠️  Press ENTER to test TURN (robot will move!)...')
            tester.test_turn()
            input('\n⚠️  Press ENTER to test REVERSE (robot will move!)...')
            tester.test_reverse()
            input('\n⚠️  Press ENTER to test STEPPER...')
            tester.test_stepper()
        else:
            tester.get_logger().error(f'Unknown test: {test_name}')
            print_usage()
        
        print('\n' + '='*60)
        print('✅ Testing complete!')
        print('='*60 + '\n')
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        # Ensure robot is stopped
        twist = Twist()
        tester.cmd_vel_pub.publish(twist)
        
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
