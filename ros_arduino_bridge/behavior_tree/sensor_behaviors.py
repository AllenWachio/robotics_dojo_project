#!/usr/bin/env python3
"""
Sensor and Actuator Behaviors for Cube Pickup and Delivery Mission
Integrates color sensor, Pi camera, stepper motor, and movement controls

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from std_msgs.msg import ColorRGBA, String, Int32, Bool
from geometry_msgs.msg import Twist
import math
import time


class ReadColorSensor(py_trees.behaviour.Behaviour):
    """
    Reads the color sensor and stores the detected color (red or blue).
    Publishes to /color_sensor/led to turn on LED, then reads RGB values.
    
    Stores result in blackboard under key 'detected_color' as 'red', 'blue', or 'unknown'.
    
    IMPORTANT: This behavior NEVER fails. If sensor times out, it returns SUCCESS
    with 'unknown' color, allowing operator to manually load the cube.
    Mission continues regardless of sensor reading.
    """
    
    def __init__(self, name="ReadColorSensor"):
        super().__init__(name)
        self.node = None
        self.color_sub = None
        self.led_pub = None
        self.latest_color = None
        self.reading_started = False
        self.start_time = None
        self.timeout = 15.0  # 15 seconds for operator to manually load cube
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to color sensor RGB data
        self.color_sub = self.node.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.color_callback,
            10
        )
        
        # Publisher to control LED
        self.led_pub = self.node.create_publisher(Bool, '/color_sensor/led', 10)
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Called when behavior starts"""
        self.latest_color = None
        self.reading_started = False
        self.start_time = time.time()
        
        # Turn on LED for color sensing
        led_msg = Bool()
        led_msg.data = True
        self.led_pub.publish(led_msg)
        self.logger.info(f"{self.name}: LED ON, reading color sensor...")
        self.reading_started = True
    
    def update(self):
        """Read color sensor and classify as red or blue"""
        if not self.reading_started:
            return py_trees.common.Status.RUNNING
        
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: ⚠️  Timeout waiting for color reading")
            self.logger.info(f"{self.name}: 🤚 Manual intervention expected - continuing with 'unknown' color")
            
            # Store 'unknown' in blackboard to allow manual loading
            blackboard = self.attach_blackboard_client(name=self.name)
            blackboard.register_key(key='detected_color', access=py_trees.common.Access.WRITE)
            blackboard.detected_color = 'unknown'
            
            # Turn off LED
            led_msg = Bool()
            led_msg.data = False
            self.led_pub.publish(led_msg)
            
            # Return SUCCESS to continue mission (manual loading assumed)
            return py_trees.common.Status.SUCCESS
        
        # Wait for color data
        if self.latest_color is None:
            return py_trees.common.Status.RUNNING
        
        # Classify color based on RGB values
        r, g, b = self.latest_color
        
        # Simple heuristic: red if R > G and R > B, blue if B > R and B > G
        if r > g and r > b and r > 0.3:  # Red dominant
            detected = 'red'
        elif b > r and b > g and b > 0.3:  # Blue dominant
            detected = 'blue'
        else:
            detected = 'unknown'
        
        # Store in blackboard
        blackboard = self.attach_blackboard_client(name=self.name)
        blackboard.register_key(key='detected_color', access=py_trees.common.Access.WRITE)
        blackboard.detected_color = detected
        
        if detected == 'unknown':
            self.logger.info(f"{self.name}: ⚠️  No clear color detected (R:{r:.2f} G:{g:.2f} B:{b:.2f}) - continuing anyway")
        else:
            self.logger.info(f"{self.name}: ✓ Detected color = {detected.upper()} (R:{r:.2f} G:{g:.2f} B:{b:.2f})")
        
        # Turn off LED
        led_msg = Bool()
        led_msg.data = False
        self.led_pub.publish(led_msg)
        
        return py_trees.common.Status.SUCCESS
    
    def color_callback(self, msg):
        """Store latest color reading"""
        self.latest_color = (msg.r, msg.g, msg.b)


class MonitorCameraForColor(py_trees.behaviour.Behaviour):
    """
    Monitors the Pi camera color detection node output.
    Looks for the color stored in blackboard['detected_color'].
    Returns SUCCESS when the target color is detected by camera.
    
    This behavior runs continuously while robot moves to Point2.
    """
    
    def __init__(self, name="MonitorCameraForColor"):
        super().__init__(name)
        self.node = None
        self.color_detection_sub = None
        self.target_color = None
        self.color_detected = False
        self.last_detection_time = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to color detection results
        # Assuming color_detection_node publishes detected colors to a topic
        # We'll monitor /color_detection/detected topic (String message with color name)
        self.color_detection_sub = self.node.create_subscription(
            String,
            '/color_detection/detected',
            self.detection_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Called when behavior starts"""
        # Get target color from blackboard
        blackboard = self.attach_blackboard_client(name=self.name)
        blackboard.register_key(key='detected_color', access=py_trees.common.Access.READ)
        self.target_color = blackboard.detected_color
        
        self.color_detected = False
        
        # Handle 'unknown' color (manual loading case)
        if self.target_color == 'unknown':
            self.logger.info(f"{self.name}: ℹ️  Color is 'unknown' (manual loading) - skipping camera monitoring")
            self.color_detected = True  # Immediately succeed
        else:
            self.logger.info(f"{self.name}: Monitoring camera for {self.target_color.upper()} color...")
    
    def update(self):
        """Check if target color has been detected"""
        if self.color_detected:
            if self.target_color == 'unknown':
                self.logger.info(f"{self.name}: ✓ Skipped (manual loading mode)")
            else:
                self.logger.info(f"{self.name}: ✓ Target color {self.target_color.upper()} detected!")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def detection_callback(self, msg):
        """Handle color detection messages from camera"""
        detected_color = msg.data.lower()
        if detected_color == self.target_color:
            self.color_detected = True
            self.last_detection_time = time.time()
            self.logger.info(f"{self.name}: ✓ Camera detected {self.target_color.upper()}")


class WaitForColorSensorClear(py_trees.behaviour.Behaviour):
    """
    Waits until the color sensor no longer detects the cube color.
    This indicates the cube has been offloaded from the robot.
    
    Monitors color sensor and returns SUCCESS when color reading drops below threshold.
    """
    
    def __init__(self, name="WaitForColorSensorClear", timeout=10.0):
        super().__init__(name)
        self.node = None
        self.color_sub = None
        self.latest_color = None
        self.timeout = timeout
        self.start_time = None
        self.target_color = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to color sensor
        self.color_sub = self.node.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.color_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Called when behavior starts"""
        self.latest_color = None
        self.start_time = time.time()
        
        # Get the target color from blackboard
        blackboard = self.attach_blackboard_client(name=self.name)
        blackboard.register_key(key='detected_color', access=py_trees.common.Access.READ)
        self.target_color = blackboard.detected_color
        
        self.logger.info(f"{self.name}: Waiting for {self.target_color.upper()} to clear from sensor...")
    
    def update(self):
        """Check if color has cleared"""
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warn(f"{self.name}: Timeout waiting for color to clear")
            return py_trees.common.Status.FAILURE
        
        # Wait for color reading
        if self.latest_color is None:
            return py_trees.common.Status.RUNNING
        
        r, g, b = self.latest_color
        
        # Check if the target color is no longer dominant
        color_cleared = False
        if self.target_color == 'red':
            # Red cleared if R is not dominant or very low
            color_cleared = (r < 0.2) or (r <= g or r <= b)
        elif self.target_color == 'blue':
            # Blue cleared if B is not dominant or very low
            color_cleared = (b < 0.2) or (b <= r or b <= g)
        else:
            # Unknown color - just check if all values are low
            color_cleared = (max(r, g, b) < 0.2)
        
        if color_cleared:
            self.logger.info(f"{self.name}: ✓ Color sensor cleared! Cube offloaded.")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def color_callback(self, msg):
        """Store latest color reading"""
        self.latest_color = (msg.r, msg.g, msg.b)


class Turn180Degrees(py_trees.behaviour.Behaviour):
    """
    Executes a 180-degree turn in place.
    Publishes Twist commands to /cmd_vel to rotate the robot.
    """
    
    def __init__(self, name="Turn180Degrees", angular_speed=0.5):
        super().__init__(name)
        self.node = None
        self.cmd_vel_pub = None
        self.angular_speed = angular_speed  # rad/s
        self.turn_started = False
        self.start_time = None
        self.duration = math.pi / angular_speed  # Time to turn 180 degrees
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.logger.info(f"{self.name}: Setup complete (angular_speed={self.angular_speed} rad/s)")
    
    def initialise(self):
        """Called when behavior starts"""
        self.turn_started = False
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Starting 180° turn (duration ~{self.duration:.1f}s)...")
    
    def update(self):
        """Execute turn"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            # Still turning
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Turn complete - stop
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.logger.info(f"{self.name}: ✓ 180° turn complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Ensure robot stops when behavior ends"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class ReverseDistance(py_trees.behaviour.Behaviour):
    """
    Reverses the robot by a specified distance (in centimeters).
    Uses simple time-based control with cmd_vel.
    
    For accurate distance control, consider integrating with odometry or encoders.
    """
    
    def __init__(self, name="ReverseDistance", distance_cm=4.0, speed=0.1):
        super().__init__(name)
        self.node = None
        self.cmd_vel_pub = None
        self.distance_cm = distance_cm
        self.speed = speed  # m/s (negative for reverse)
        self.start_time = None
        self.duration = (distance_cm / 100.0) / abs(speed)  # time to travel distance
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.logger.info(f"{self.name}: Setup complete (distance={self.distance_cm}cm, speed={self.speed}m/s)")
    
    def initialise(self):
        """Called when behavior starts"""
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Reversing {self.distance_cm}cm (duration ~{self.duration:.2f}s)...")
    
    def update(self):
        """Execute reverse"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            # Still reversing
            twist = Twist()
            twist.linear.x = -abs(self.speed)  # Negative for reverse
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Reverse complete - stop
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.logger.info(f"{self.name}: ✓ Reverse complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Ensure robot stops when behavior ends"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class ActivateStepperMotor(py_trees.behaviour.Behaviour):
    """
    Activates the stepper motor for offloading the cube.
    Sends command to /stepper/command topic.
    
    Command format: "rpm:distance_mm:flag"
    Example: "-25:400:0" for offloading motion
    """
    
    def __init__(self, name="ActivateStepperMotor", rpm=-25, distance_mm=400, flag=0, wait_time=2.0):
        super().__init__(name)
        self.node = None
        self.stepper_pub = None
        self.rpm = rpm
        self.distance_mm = distance_mm
        self.flag = flag
        self.wait_time = wait_time  # Time to wait for stepper to complete
        self.start_time = None
        self.command_sent = False
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for stepper commands
        self.stepper_pub = self.node.create_publisher(String, '/stepper/command', 10)
        
        self.logger.info(f"{self.name}: Setup complete (rpm={self.rpm}, dist={self.distance_mm}mm)")
    
    def initialise(self):
        """Called when behavior starts"""
        self.command_sent = False
        self.start_time = time.time()
    
    def update(self):
        """Send stepper command and wait"""
        if not self.command_sent:
            # Send stepper command
            command = String()
            command.data = f"{self.rpm}:{self.distance_mm}:{self.flag}"
            self.stepper_pub.publish(command)
            
            self.logger.info(f"{self.name}: Stepper command sent: {command.data}")
            self.command_sent = True
            self.start_time = time.time()
        
        # Wait for stepper to complete
        elapsed = time.time() - self.start_time
        if elapsed < self.wait_time:
            return py_trees.common.Status.RUNNING
        else:
            self.logger.info(f"{self.name}: ✓ Stepper motor operation complete")
            return py_trees.common.Status.SUCCESS


class StopRobot(py_trees.behaviour.Behaviour):
    """
    Stops the robot by publishing zero velocity to /cmd_vel.
    Useful for ensuring robot is stopped before sensor readings.
    """
    
    def __init__(self, name="StopRobot", duration=0.5):
        super().__init__(name)
        self.node = None
        self.cmd_vel_pub = None
        self.duration = duration  # How long to hold stop command
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Called when behavior starts"""
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Stopping robot...")
    
    def update(self):
        """Send stop command"""
        # Send stop command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Wait for duration
        elapsed = time.time() - self.start_time
        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            self.logger.info(f"{self.name}: ✓ Robot stopped")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Final stop command"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


# =====================================================
# NEW BEHAVIORS FOR DISEASE DETECTION PHASE
# =====================================================

class MoveRelativeDistance(py_trees.behaviour.Behaviour):
    """
    Move robot a relative distance in X (forward/backward) and Y (left/right).
    Uses cmd_vel with time-based control.
    
    Positive X = forward, Negative X = backward
    Positive Y = left, Negative Y = right
    """
    
    def __init__(self, name, distance_x_m=0.0, distance_y_m=0.0, speed=0.15):
        super().__init__(name)
        self.distance_x = distance_x_m  # meters
        self.distance_y = distance_y_m  # meters
        self.speed = speed  # m/s
        self.node = None
        self.cmd_vel_pub = None
        self.start_time = None
        self.phase = "x"  # "x" for forward/back, "y" for left/right
        self.x_complete = False
        self.y_complete = False
        
    def setup(self, **kwargs):
        """Initialize ROS2 publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Reset state"""
        self.start_time = time.time()
        self.phase = "x" if self.distance_x != 0 else "y"
        self.x_complete = (self.distance_x == 0)
        self.y_complete = (self.distance_y == 0)
        self.logger.info(f"{self.name}: Moving X={self.distance_x}m, Y={self.distance_y}m")
    
    def update(self):
        """Execute relative movement"""
        twist = Twist()
        
        # Phase 1: X movement (forward/backward)
        if not self.x_complete:
            duration_x = abs(self.distance_x) / self.speed
            elapsed = time.time() - self.start_time
            
            if elapsed < duration_x:
                twist.linear.x = self.speed if self.distance_x > 0 else -self.speed
                self.cmd_vel_pub.publish(twist)
                return py_trees.common.Status.RUNNING
            else:
                # Stop and transition to Y phase
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                self.x_complete = True
                self.phase = "y"
                self.start_time = time.time()  # Reset timer
                time.sleep(0.2)  # Brief pause
                self.logger.info(f"{self.name}: X movement complete")
        
        # Phase 2: Y movement (left/right - strafe)
        if not self.y_complete and self.x_complete:
            duration_y = abs(self.distance_y) / self.speed
            elapsed = time.time() - self.start_time
            
            if elapsed < duration_y:
                twist.linear.y = self.speed if self.distance_y > 0 else -self.speed
                self.cmd_vel_pub.publish(twist)
                return py_trees.common.Status.RUNNING
            else:
                # Stop - movement complete
                twist.linear.y = 0.0
                self.cmd_vel_pub.publish(twist)
                self.y_complete = True
                self.logger.info(f"{self.name}: ✓ Relative movement complete")
                return py_trees.common.Status.SUCCESS
        
        # Both movements complete
        if self.x_complete and self.y_complete:
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Stop robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class Turn90Left(py_trees.behaviour.Behaviour):
    """
    Turn robot 90 degrees to the left (counterclockwise).
    """
    
    def __init__(self, name="Turn90Left", angular_speed=0.5):
        super().__init__(name)
        self.angular_speed = angular_speed  # rad/s
        self.node = None
        self.cmd_vel_pub = None
        self.start_time = None
        # 90 degrees = π/2 radians
        self.target_angle = math.pi / 2.0
        self.duration = self.target_angle / self.angular_speed
        
    def setup(self, **kwargs):
        """Initialize ROS2 publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info(f"{self.name}: Setup complete (duration: {self.duration:.2f}s)")
    
    def initialise(self):
        """Start turn"""
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Turning 90° left...")
    
    def update(self):
        """Execute turn"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            twist = Twist()
            twist.angular.z = self.angular_speed  # Positive = left
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Stop rotation
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.logger.info(f"{self.name}: ✓ 90° turn complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Stop rotation"""
        twist = Twist()
        twist.angular.z = 0.0
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)


class ActivateCameraServo(py_trees.behaviour.Behaviour):
    """
    Activates camera servo to adjust angle for better plant leaf detection.
    Publishes to /camera_servo/command topic.
    
    Typical angles:
    - 0° = looking down
    - 45° = angled view
    - 90° = looking forward
    """
    
    def __init__(self, name="ActivateCameraServo", target_angle=45):
        super().__init__(name)
        self.target_angle = target_angle  # degrees
        self.node = None
        self.servo_pub = None
        self.servo_feedback_sub = None
        self.current_angle = None
        self.command_sent = False
        self.start_time = None
        self.timeout = 3.0  # 3 seconds to reach position
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for servo command
        self.servo_pub = self.node.create_publisher(
            Int32,
            '/camera_servo/command',
            10
        )
        
        # Subscriber for servo feedback
        self.servo_feedback_sub = self.node.create_subscription(
            Int32,
            '/camera_servo/angle',
            self.servo_feedback_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete (target: {self.target_angle}°)")
    
    def servo_feedback_callback(self, msg):
        """Receive current servo angle"""
        self.current_angle = msg.data
    
    def initialise(self):
        """Send servo command"""
        self.command_sent = False
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Moving camera servo to {self.target_angle}°...")
    
    def update(self):
        """Monitor servo until position reached"""
        # Send command once
        if not self.command_sent:
            msg = Int32()
            msg.data = self.target_angle
            self.servo_pub.publish(msg)
            self.command_sent = True
            self.logger.info(f"{self.name}: Command sent")
        
        # Check if reached target (within 5 degrees tolerance)
        if self.current_angle is not None:
            error = abs(self.current_angle - self.target_angle)
            if error <= 5:
                self.logger.info(f"{self.name}: ✓ Servo at {self.current_angle}° (target: {self.target_angle}°)")
                return py_trees.common.Status.SUCCESS
        
        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            self.logger.warning(f"{self.name}: Timeout - assuming position reached")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class WaitForDiseaseDetection(py_trees.behaviour.Behaviour):
    """
    Waits for potato disease detection result from /inference_result topic.
    Stores result in blackboard under key 'disease_detection_result'.
    
    Expected values: 'Early Blight', 'Late Blight', 'Healthy', etc.
    """
    
    def __init__(self, name="WaitForDiseaseDetection", timeout=10.0):
        super().__init__(name)
        self.timeout = timeout
        self.node = None
        self.detection_sub = None
        self.detection_result = None
        self.start_time = None
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 subscription and blackboard"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to disease detection results
        self.detection_sub = self.node.create_subscription(
            String,
            '/inference_result',
            self.detection_callback,
            10
        )
        
        # Register blackboard key
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='disease_detection_result',
            access=py_trees.common.Access.WRITE
        )
        
        self.logger.info(f"{self.name}: Setup complete (timeout: {self.timeout}s)")
    
    def detection_callback(self, msg):
        """Receive disease detection result"""
        self.detection_result = msg.data
        self.logger.info(f"{self.name}: Received detection: {self.detection_result}")
    
    def initialise(self):
        """Reset state"""
        self.detection_result = None
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Waiting for disease detection result...")
    
    def update(self):
        """Wait for detection result"""
        # Check if result received
        if self.detection_result is not None:
            # Store in blackboard
            self.blackboard.set('disease_detection_result', self.detection_result)
            self.logger.info(f"{self.name}: ✓ Detection complete: {self.detection_result}")
            return py_trees.common.Status.SUCCESS
        
        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed >= self.timeout:
            self.logger.warning(f"{self.name}: ⚠️  Detection timeout ({self.timeout}s) - cancelling and moving on")
            # Store 'timeout' as result
            self.blackboard.set('disease_detection_result', 'timeout')
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


# =====================================================
# OFFLOAD RETRY BEHAVIORS
# =====================================================

class ActivateTipperServo(py_trees.behaviour.Behaviour):
    """
    Activates tipper servo to assist with offloading.
    Used as a retry mechanism when stepper motor offload fails.
    Publishes to /tipper_servo/command topic.
    
    Typical angles:
    - 0° = flat/resting position
    - 90° = tipped up for offloading
    """
    
    def __init__(self, name="ActivateTipperServo", tip_angle=90, duration=3.0):
        super().__init__(name)
        self.tip_angle = tip_angle  # degrees
        self.duration = duration    # how long to hold tip position
        self.node = None
        self.servo_pub = None
        self.command_sent = False
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for tipper servo command
        self.servo_pub = self.node.create_publisher(
            Int32,
            '/tipper_servo/command',
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete (tip angle: {self.tip_angle}°)")
    
    def initialise(self):
        """Send tip command"""
        self.command_sent = False
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Activating tipper servo to {self.tip_angle}°...")
    
    def update(self):
        """Send tip command and wait"""
        # Send command once
        if not self.command_sent:
            msg = Int32()
            msg.data = self.tip_angle
            self.servo_pub.publish(msg)
            self.command_sent = True
            self.logger.info(f"{self.name}: Tipper servo command sent")
        
        # Wait for duration
        elapsed = time.time() - self.start_time
        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            # Return to flat position
            msg = Int32()
            msg.data = 0  # Flat
            self.servo_pub.publish(msg)
            self.logger.info(f"{self.name}: ✓ Tipper servo operation complete (returned to flat)")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Return servo to flat position"""
        if self.servo_pub:
            msg = Int32()
            msg.data = 0  # Flat
            self.servo_pub.publish(msg)


class OffloadWithRetry(py_trees.behaviour.Behaviour):
    """
    Composite behavior that checks if offload succeeded after stepper motor.
    If color sensor still detects cube, activates tipper servo + stepper again.
    
    Sequence:
    1. Check color sensor
    2. If clear → SUCCESS
    3. If still loaded → Activate tipper servo + stepper motor (retry)
    4. Check again → SUCCESS or FAILURE
    """
    
    def __init__(self, name="OffloadWithRetry", check_timeout=15.0):
        super().__init__(name)
        self.check_timeout = check_timeout
        self.node = None
        self.color_sub = None
        self.stepper_pub = None
        self.latest_color = None
        self.retry_attempted = False
        self.phase = "initial_check"  # initial_check, retry, wait_for_stepper, final_check
        self.start_time = None
        self.stepper_start_time = None
        self.stepper_duration = 5.0  # Non-blocking wait time for stepper
        
        # Tipper servo behavior
        self.tipper_behavior = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to color sensor
        self.color_sub = self.node.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.color_callback,
            10
        )
        
        # Publisher for stepper
        self.stepper_pub = self.node.create_publisher(
            String,
            '/stepper/command',
            10
        )
        
        # Create tipper servo behavior
        self.tipper_behavior = ActivateTipperServo(
            f"{self.name}_Tipper",
            tip_angle=90,
            duration=3.0
        )
        self.tipper_behavior.setup(**kwargs)
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def color_callback(self, msg):
        """Receive color sensor data"""
        self.latest_color = (msg.r, msg.g, msg.b)
    
    def is_cube_detected(self):
        """Check if cube is still on robot (color sensor sees color)"""
        if self.latest_color is None:
            return False
        
        r, g, b = self.latest_color
        total = r + g + b
        
        if total < 0.01:  # Very dark = no object
            return False
        
        # Normalize
        if total > 0:
            r = r / total
            g = g / total
            b = b / total
        
        # Check if any strong color detected (cube present)
        if r > 0.3 or b > 0.3:
            return True
        
        return False
    
    def initialise(self):
        """Reset state"""
        self.retry_attempted = False
        self.phase = "initial_check"
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Checking if offload succeeded...")
    
    def update(self):
        """Check sensor and retry if needed"""
        
        # PHASE 1: Initial check after stepper
        if self.phase == "initial_check":
            elapsed = time.time() - self.start_time
            
            if self.is_cube_detected():
                if elapsed < self.check_timeout:
                    # Still detecting, keep waiting
                    return py_trees.common.Status.RUNNING
                else:
                    # Timeout - cube still there, need retry!
                    self.logger.warning(f"{self.name}: ⚠️  Cube still detected after {self.check_timeout}s!")
                    self.logger.info(f"{self.name}: 🔄 Initiating RETRY with tipper servo...")
                    self.phase = "retry"
                    self.tipper_behavior.initialise()
                    return py_trees.common.Status.RUNNING
            else:
                # Cube cleared!
                self.logger.info(f"{self.name}: ✓ Offload successful! Color sensor clear.")
                return py_trees.common.Status.SUCCESS
        
        # PHASE 2: Execute retry (tipper servo + stepper)
        elif self.phase == "retry":
            # Run tipper servo
            tipper_status = self.tipper_behavior.update()
            
            if tipper_status == py_trees.common.Status.RUNNING:
                return py_trees.common.Status.RUNNING
            elif tipper_status == py_trees.common.Status.SUCCESS:
                # Tipper done, activate stepper again
                self.logger.info(f"{self.name}: Activating stepper motor again...")
                stepper_msg = String()
                stepper_msg.data = "-25:400:0"  # Same as before
                self.stepper_pub.publish(stepper_msg)
                
                # Move to wait phase (non-blocking wait for stepper)
                self.phase = "wait_for_stepper"
                self.stepper_start_time = time.time()
                return py_trees.common.Status.RUNNING
        
        # PHASE 2.5: Wait for stepper to complete (non-blocking)
        elif self.phase == "wait_for_stepper":
            elapsed = time.time() - self.stepper_start_time
            if elapsed < self.stepper_duration:
                # Still waiting for stepper
                return py_trees.common.Status.RUNNING
            else:
                # Stepper done, move to final check
                self.phase = "final_check"
                self.start_time = time.time()
                self.logger.info(f"{self.name}: Retry complete, checking sensor again...")
                return py_trees.common.Status.RUNNING
        
        # PHASE 3: Final check after retry
        elif self.phase == "final_check":
            elapsed = time.time() - self.start_time
            
            if self.is_cube_detected():
                if elapsed < self.check_timeout:
                    return py_trees.common.Status.RUNNING
                else:
                    # Still stuck after retry
                    self.logger.error(f"{self.name}: ❌ Offload FAILED even after retry!")
                    return py_trees.common.Status.FAILURE
            else:
                self.logger.info(f"{self.name}: ✓ Retry successful! Cube offloaded.")
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        if self.tipper_behavior:
            self.tipper_behavior.terminate(new_status)
