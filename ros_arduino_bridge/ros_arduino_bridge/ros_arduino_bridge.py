#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String, ColorRGBA, Bool
from std_srvs.srv import Trigger
import serial
import threading
import time
import math
import re
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import select


class ROSArduinoBridge(Node):
    def __init__(self):
        super().__init__("ros_arduino_bridge")

        # Parameters
        self.declare_parameter("serial_port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("base_width", 0.249000)  # Effective track width (20.8cm + 4.1cm wheel width)
        self.declare_parameter("wheel_radius", 0.0425)
        self.declare_parameter(
            "encoder_ticks_per_rev", 447
        )  # The measured ticks per revolution (calibrated value)

        # Max speeds for mapping to PWM
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        
        # TF publishing control (disable when using EKF for sensor fusion)
        self.declare_parameter("publish_tf", True)

        # Get parameters
        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.base_width = self.get_parameter("base_width").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.encoder_ticks_per_rev = self.get_parameter("encoder_ticks_per_rev").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        # Frames
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        # TF control
        self.publish_tf_enabled = self.get_parameter("publish_tf").value

        # Serial communication
        self.serial = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # Robot state
        self.encoder_counts = [0, 0, 0, 0]  # M1, M2, M3, M4
        self.last_encoder_counts = [0, 0, 0, 0]
        self.imu_angle = 0.0
        self.ultrasonic_ranges = [0.0, 0.0]  # Left, Right (in meters)
        self.robot_state = 2  # Default to stationary
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_update = self.get_clock().now()
        # Throttle warnings for invalid encoder data
        self._last_invalid_encoder_warn = 0.0
        # Throttle warnings for color sensor (low values)
        self._last_color_sensor_warn = 0.0
        
        # IMU calibration state
        self.imu_calibrated = False
        self.imu_gyro_bias = [0.0, 0.0, 0.0]  # Gyro bias (raw units)
        self.imu_calibration_samples = 0
        
        # Start calibration after a short delay (let Arduino stabilize)
        self.create_timer(3.0, self.calibrate_imu_once)

        # Servo state
        self.camera_servo_angle = 50  # Initial position from Arduino
        self.tipper_servo_angle = 170  # Initial position from Arduino

        # Color sensor state
        self.color_rgb = [0, 0, 0]  # R, G, B raw values
        self.color_led_state = False

        # Stepper state
        self.stepper_active = False
        self.stepper_last_command = ""

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        # IMU publisher (Arduino command 'z' returns yaw in degrees)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        # Ultrasonic publishers temporarily disabled (encoders-only mode)
        # self.ultrasonic_left_pub = self.create_publisher(Range, "ultrasonic/left", 10)
        # self.ultrasonic_right_pub = self.create_publisher(Range, "ultrasonic/right", 10)
        self.robot_state_pub = self.create_publisher(Int32, "robot_state", 10)
        self.raw_encoder_pub = self.create_publisher(String, "raw_encoders", 10)
        # Joint states for visualization (RViz / joint_state_publisher expects /joint_states)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        
        # New publishers for additional sensors/actuators (with debug topics)
        self.color_sensor_pub = self.create_publisher(ColorRGBA, "color_sensor/rgb", 10)
        self.color_sensor_raw_pub = self.create_publisher(String, "color_sensor/raw", 10)  # Debug topic
        self.stepper_status_pub = self.create_publisher(Bool, "stepper/active", 10)
        self.stepper_debug_pub = self.create_publisher(String, "stepper/debug", 10)  # Debug topic
        self.camera_servo_feedback_pub = self.create_publisher(Int32, "camera_servo/angle", 10)
        self.camera_servo_debug_pub = self.create_publisher(String, "camera_servo/debug", 10)  # Debug topic
        self.tipper_servo_feedback_pub = self.create_publisher(Int32, "tipper_servo/angle", 10)
        self.tipper_servo_debug_pub = self.create_publisher(String, "tipper_servo/debug", 10)  # Debug topic

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        
        # New subscribers for servos and stepper
        self.camera_servo_sub = self.create_subscription(
            Int32, "camera_servo/command", self.camera_servo_callback, 10
        )
        self.tipper_servo_sub = self.create_subscription(
            Int32, "tipper_servo/command", self.tipper_servo_callback, 10
        )
        self.stepper_sub = self.create_subscription(
            String, "stepper/command", self.stepper_callback, 10
        )
        self.color_led_sub = self.create_subscription(
            Bool, "color_sensor/led", self.color_led_callback, 10
        )

        # Services
        self.get_color_srv = self.create_service(
            Trigger, "get_color", self.handle_get_color
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.data_timer = self.create_timer(0.05, self.update_sensor_data)  # 20Hz
        self.odom_timer = self.create_timer(0.1, self.update_odometry)  # 10Hz
        self.state_timer = self.create_timer(1.0, self.publish_robot_state)  # 1Hz
        self.color_timer = self.create_timer(0.2, self.read_color_sensor)  # 5Hz
        self.imu_timer = self.create_timer(0.2, self._request_imu)  # 5Hz IMU polling

        self.get_logger().info("ROS Arduino Bridge node started")

    def connect_serial(self):
        """Connect to Arduino serial port"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                write_timeout=1.0,
            )
            time.sleep(2)  # Wait for Arduino to reset
            # Clear any startup messages from the Arduino (e.g., IMU or debug prints)
            try:
                self.serial.reset_input_buffer()
            except Exception:
                # Older pyserial versions may not have reset_input_buffer
                pass
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")

            # Reset encoders on startup
            self.send_command("r")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            # Schedule reconnection attempt
            self.create_timer(5.0, self.reconnect_serial)

    def reconnect_serial(self):
        """Attempt to reconnect to Arduino"""
        if self.serial is None or not self.serial.is_open:
            self.get_logger().info("Attempting to reconnect to Arduino...")
            self.connect_serial()

    def calibrate_imu_once(self):
        """One-shot IMU calibration - runs once at startup"""
        if self.imu_calibrated:
            return  # Already calibrated
        
        self.get_logger().info("Starting IMU calibration - keep robot STATIONARY for 5 seconds...")
        
        samples = []
        start_time = time.time()
        
        # Collect samples for 5 seconds
        while time.time() - start_time < 5.0:
            if self.send_command('i'):
                time.sleep(0.05)  # Wait for Arduino to respond
                line = self.read_serial_line()
                if line:
                    try:
                        values = list(map(float, line.split()))
                        if len(values) == 9:
                            # Extract raw gyro values (indices 3, 4, 5)
                            samples.append([values[3], values[4], values[5]])
                    except (ValueError, IndexError):
                        pass
            time.sleep(0.1)  # 10Hz sampling
        
        if len(samples) >= 10:
            # Calculate average gyro bias
            avg_gx = sum(s[0] for s in samples) / len(samples)
            avg_gy = sum(s[1] for s in samples) / len(samples)
            avg_gz = sum(s[2] for s in samples) / len(samples)
            
            self.imu_gyro_bias = [avg_gx, avg_gy, avg_gz]
            self.imu_calibrated = True
            self.imu_calibration_samples = len(samples)
            
            self.get_logger().info(
                f"IMU calibration complete ({len(samples)} samples). "
                f"Gyro bias: X={avg_gx:.2f} Y={avg_gy:.2f} Z={avg_gz:.2f} (raw units)"
            )
        else:
            self.get_logger().warning(
                f"IMU calibration failed - only {len(samples)} samples. "
                "Proceeding with zero bias (gyro drift may occur)"
            )
            self.imu_calibrated = True  # Don't retry forever

    def send_command(self, command):
        """Send command to Arduino with thread safety and verification"""
        with self.serial_lock:
            if self.serial and self.serial.is_open:
                try:
                    # Clear any stale data in input buffer first
                    if self.serial.in_waiting > 0:
                        stale = self.serial.read_all().decode('utf-8', errors='ignore')
                        self.get_logger().warn(f"Cleared stale buffer: '{stale}'")
                    
                    # Send command with carriage return
                    full_command = command + "\r"
                    self.serial.write(full_command.encode())
                    self.serial.flush()
                    
                    # Log for motor commands (debug)
                    if command.startswith('m '):
                        self.get_logger().info(f"→ SENT: '{full_command.strip()}'")
                    
                    return True
                except Exception as e:
                    self.get_logger().error(f"Error sending command: {e}")
                    self.serial.close()
                    self.serial = None
        return False

    def read_serial_line(self):
        """Read a line from serial with timeout"""
        if self.serial and self.serial.is_open:
            try:
                # Use select to check if data is available (non-blocking)
                ready, _, _ = select.select([self.serial], [], [], 0.1)
                if ready:
                    line = self.serial.readline().decode("utf-8").strip()
                    if line:
                        return line
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")
                self.serial.close()
                self.serial = None
        return None

    def _request_imu(self):
        """Send the IMU request command to the Arduino ('i')."""
        try:
            if self.serial and self.serial.is_open:
                self.send_command('i')
        except Exception:
            # Keep silent to avoid timer storms
            pass

    def _handle_imu_line(self, line: str):
        """
        Parse IMU line from Arduino and publish sensor_msgs/Imu.
        Expected format (9 space-separated values):
          "yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z"
        Example: "45.30 2.10 -1.50 23.0 -4.1 1.2 512.0 514.2 490.1"
        
        Arduino sends ALL VALUES IN DEGREES:
        - yaw: -180° to +180° (negative=left, positive=right) ← Standard convention
        - pitch, roll: degrees (from DMP)
        - gyro_x/y/z: RAW register values (need conversion to rad/s)
        - accel_x/y/z: RAW register values (need conversion to m/s²)
        """
        s = line.strip()
        
        # Try to parse 9 space-separated float values
        try:
            values = list(map(float, s.split()))
            if len(values) != 9:
                return  # Not the expected IMU format
        except (ValueError, AttributeError):
            return  # Can't parse as floats
        
        # Extract values (ALL IN DEGREES from Arduino!)
        yaw_deg, pitch_deg, roll_deg = values[0], values[1], values[2]
        gyro_x_raw, gyro_y_raw, gyro_z_raw = values[3], values[4], values[5]
        accel_x_raw, accel_y_raw, accel_z_raw = values[6], values[7], values[8]
        
        # Apply calibration bias (subtract offset measured at startup)
        if self.imu_calibrated:
            gyro_x_raw -= self.imu_gyro_bias[0]
            gyro_y_raw -= self.imu_gyro_bias[1]
            gyro_z_raw -= self.imu_gyro_bias[2]
        
        # CRITICAL: Convert angles from DEGREES to RADIANS for ROS!
        # Arduino convention: yaw -180° to +180° (negative=left, positive=right)
        # ROS convention: radians, same sign convention
        yaw = math.radians(yaw_deg)      # -π to +π
        pitch = math.radians(pitch_deg)  # -π to +π
        roll = math.radians(roll_deg)    # -π to +π
        
        # Convert RAW gyro to rad/s (for logging/debugging only)
        # MPU6050 with FS_SEL=0 (±250°/s): LSB sensitivity = 131 LSB/(°/s)
        gyro_scale = 131.0  # LSB per degree/second
        gyro_x_dps = gyro_x_raw / gyro_scale  # degrees per second
        gyro_y_dps = gyro_y_raw / gyro_scale
        gyro_z_dps = gyro_z_raw / gyro_scale
        gyro_x = math.radians(gyro_x_dps)  # convert to rad/s
        gyro_y = math.radians(gyro_y_dps)
        gyro_z = math.radians(gyro_z_dps)  # CRITICAL: Define gyro_z to prevent NameError
        
        # NOTE: We are NOT using angular velocity from IMU in the EKF
        # The EKF uses ONLY yaw orientation (imu0_config: only yaw enabled)
        # This prevents gyro drift from affecting the filtered odometry
        # Gyro is only for logging/debugging
        
        # Convert RAW accel to m/s²
        # MPU6050 with AFS_SEL=0 (±2g): raw / 16384.0 = g
        # Then convert g to m/s²
        accel_scale = 16384.0  # LSB/g for ±2g range
        g_to_ms2 = 9.80665  # Standard gravity
        accel_x = (accel_x_raw / accel_scale) * g_to_ms2
        accel_y = (accel_y_raw / accel_scale) * g_to_ms2
        accel_z = (accel_z_raw / accel_scale) * g_to_ms2
        
        # Compute quaternion from roll, pitch, yaw (ZYX convention)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        imu_msg = Imu()
        
        # Orientation quaternion from roll, pitch, yaw
        imu_msg.orientation.x = float(qx)
        imu_msg.orientation.y = float(qy)
        imu_msg.orientation.z = float(qz)
        imu_msg.orientation.w = float(qw)
        
        # Angular velocity (rad/s)
        # CRITICAL: Set to ZERO since EKF has angular velocity DISABLED
        # Publishing non-zero values here can cause the EKF to drift even when disabled
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        
        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        imu_msg.linear_acceleration.z = float(accel_z)
        
        # Set covariance matrices - tuned for MPU6050 DMP
        # Orientation covariance: DMP fusion is very reliable
        imu_msg.orientation_covariance = [
            0.001, 0.0, 0.0,    # Roll: excellent (DMP fusion)
            0.0, 0.001, 0.0,    # Pitch: excellent (DMP fusion)
            0.0, 0.0, 0.002     # Yaw: slightly higher (magnetometer drift if present)
        ]
        
        # Angular velocity covariance: raw gyro has some drift
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,     # Gyro X: medium noise
            0.0, 0.02, 0.0,     # Gyro Y: medium noise
            0.0, 0.0, 0.02      # Gyro Z: medium noise (critical for turns)
        ]
        
        # Linear acceleration covariance: very noisy on small robots
        imu_msg.linear_acceleration_covariance = [
            0.5, 0.0, 0.0,      # Accel X: high noise from vibration
            0.0, 0.5, 0.0,      # Accel Y: high noise from vibration
            0.0, 0.0, 0.5       # Accel Z: high noise from vibration
        ]
        
        try:
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'  # IMU has its own frame for proper TF tree
            self.imu_pub.publish(imu_msg)
            self.get_logger().debug(
                f"IMU: yaw={yaw_deg:.2f}° pitch={pitch_deg:.2f}° roll={roll_deg:.2f}° | "
                f"gyro_z={math.degrees(gyro_z):.2f}°/s (computed from yaw diff) | accel_z={accel_z:.2f}m/s²"
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to publish IMU: {e}")

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor speeds for 4WD with proper center rotation"""
        linear = msg.linear.x
        angular = msg.angular.z

        # CRITICAL FIX: For 4-wheel skid-steer, all wheels must participate in rotation!
        # Previous code had issues with turning - motors weren't receiving correct commands
        
        # Calculate wheel speeds for differential drive
        # For rotation about center: left wheels move opposite to right wheels
        # For forward motion: all wheels move same direction
        # For combined motion: superposition of both
        
        # Base speeds from linear motion (same for all wheels)
        left_speed = linear
        right_speed = linear
        
        # Add rotational component (opposite for left/right)
        # CRITICAL: Use full base_width, not half, for proper center rotation
        rotation_speed = angular * (self.base_width / 2.0)
        
        left_speed -= rotation_speed   # Left wheels slower/reverse when turning right
        right_speed += rotation_speed  # Right wheels faster/forward when turning right
        
        # Convert m/s to PWM (-255 to +255)
        max_lin = float(self.max_linear_speed) if self.max_linear_speed != 0 else 0.5
        
        # Calculate raw PWM for all wheels (left and right have same speed on each side)
        front_left_pwm = int((left_speed / max_lin) * 255)
        front_right_pwm = int((right_speed / max_lin) * 255)
        rear_left_pwm = int((left_speed / max_lin) * 255)
        rear_right_pwm = int((right_speed / max_lin) * 255)
        
        # CRITICAL: Dead zone compensation - motors need minimum PWM to overcome friction
        # If PWM is below threshold, either set to 0 (stop) or boost to minimum
        MIN_PWM = 40  # Minimum PWM to overcome motor friction/dead zone
        
        def apply_dead_zone(pwm_value):
            """Apply dead zone compensation to PWM value"""
            if pwm_value == 0:
                return 0
            elif 0 < pwm_value < MIN_PWM:
                return MIN_PWM  # Boost weak positive to minimum
            elif -MIN_PWM < pwm_value < 0:
                return -MIN_PWM  # Boost weak negative to minimum
            else:
                return pwm_value  # Strong enough, use as-is
        
        # Apply dead zone compensation to all wheels
        front_left_pwm = apply_dead_zone(front_left_pwm)
        front_right_pwm = apply_dead_zone(front_right_pwm)
        rear_left_pwm = apply_dead_zone(rear_left_pwm)
        rear_right_pwm = apply_dead_zone(rear_right_pwm)
        
        # Clamp all PWM values independently to valid range
        front_left_pwm = max(-255, min(255, front_left_pwm))
        front_right_pwm = max(-255, min(255, front_right_pwm))
        rear_left_pwm = max(-255, min(255, rear_left_pwm))
        rear_right_pwm = max(-255, min(255, rear_right_pwm))
        
        # CRITICAL: Send all 4 motor values explicitly
        # Format: m M1:M2:M3:M4 where M1=FL, M2=FR, M3=RL, M4=RR
        command = f"m {front_left_pwm}:{front_right_pwm}:{rear_left_pwm}:{rear_right_pwm}"
        
        # Send command
        success = self.send_command(command)
        
        # Enhanced debug logging - ALWAYS log to see actual PWM values
        if abs(linear) > 0.01 or abs(angular) > 0.01:  # Any motion command
            self.get_logger().info(
                f"CMD_VEL: lin={linear:.3f} ang={angular:.3f} | "
                f"Speeds: L={left_speed:.3f} R={right_speed:.3f} | "
                f"PWM: M1(FL)={front_left_pwm:4d} M2(FR)={front_right_pwm:4d} M3(RL)={rear_left_pwm:4d} M4(RR)={rear_right_pwm:4d} | "
                f"Command: '{command}' sent={success}"
            )
        else:
            self.get_logger().debug(
                f"cmd_vel: lin={linear:.3f} ang={angular:.3f} | "
                f"PWM: FL={front_left_pwm} FR={front_right_pwm} RL={rear_left_pwm} RR={rear_right_pwm}"
            )

    def camera_servo_callback(self, msg):
        """Handle camera servo position commands"""
        angle = max(0, min(180, msg.data))  # Clamp to 0-180
        self.camera_servo_angle = angle
        # Command format: s <servo_index> <angle>
        # Camera servo is index 0 (CAMERA_SERVO_INDEX)
        command = f"s 0 {angle}"
        if self.send_command(command):
            self.get_logger().debug(f"Camera servo set to {angle} degrees")
            # Publish feedback
            feedback = Int32()
            feedback.data = angle
            self.camera_servo_feedback_pub.publish(feedback)
            # Publish debug info
            debug_msg = String()
            debug_msg.data = f"Camera servo commanded to {angle}°, sent: '{command}'"
            self.camera_servo_debug_pub.publish(debug_msg)

    def tipper_servo_callback(self, msg):
        """Handle tipper servo position commands"""
        angle = max(0, min(180, msg.data))  # Clamp to 0-180
        self.tipper_servo_angle = angle
        # Command format: s <servo_index> <angle>
        # Tipper servo is index 1 (TIPPER_SERVO_INDEX)
        command = f"s 1 {angle}"
        if self.send_command(command):
            self.get_logger().debug(f"Tipper servo set to {angle} degrees")
            # Publish feedback
            feedback = Int32()
            feedback.data = angle
            self.tipper_servo_feedback_pub.publish(feedback)
            # Publish debug info
            debug_msg = String()
            debug_msg.data = f"Tipper servo commanded to {angle}°, sent: '{command}'"
            self.tipper_servo_debug_pub.publish(debug_msg)

    def stepper_callback(self, msg):
        """Handle stepper motor commands
        Expected format: "rpm:distance_mm:flag"
        Example: "-25:400:0" - move at 25 RPM for 400mm
                 "0:0:1" - return to zero position
        """
        try:
            parts = msg.data.split(':')
            if len(parts) != 3:
                self.get_logger().error(f"Invalid stepper command format: {msg.data}")
                debug_msg = String()
                debug_msg.data = f"ERROR: Invalid format '{msg.data}', expected 'rpm:distance:flag'"
                self.stepper_debug_pub.publish(debug_msg)
                return
            
            rpm = int(parts[0])
            distance_mm = int(parts[1])
            flag = int(parts[2])
            
            # Command format: q rpm:distance:flag
            command = f"q {rpm}:{distance_mm}:{flag}"
            self.stepper_last_command = msg.data
            
            if self.send_command(command):
                self.stepper_active = True
                self.get_logger().info(
                    f"Stepper command: RPM={rpm}, Distance={distance_mm}mm, Flag={flag}"
                )
                
                # Publish status
                status_msg = Bool()
                status_msg.data = True
                self.stepper_status_pub.publish(status_msg)
                
                # Publish debug info
                debug_msg = String()
                debug_msg.data = f"Stepper: rpm={rpm}, dist={distance_mm}mm, flag={flag}, cmd='{command}'"
                self.stepper_debug_pub.publish(debug_msg)
        
        except ValueError as e:
            self.get_logger().error(f"Error parsing stepper command: {e}")
            debug_msg = String()
            debug_msg.data = f"ERROR parsing: {str(e)}"
            self.stepper_debug_pub.publish(debug_msg)

    def color_led_callback(self, msg):
        """Control color sensor LED"""
        self.color_led_state = msg.data
        # Command format: v 1 (LED on) or v 0 (LED off)
        led_val = '1' if msg.data else '0'
        command = f"v {led_val}"
        if self.send_command(command):
            self.get_logger().debug(f"Color sensor LED: {'ON' if msg.data else 'OFF'}")
            # Publish debug info
            debug_msg = String()
            debug_msg.data = f"Color LED: {'ON' if msg.data else 'OFF'}, cmd='{command}'"
            self.color_sensor_raw_pub.publish(debug_msg)

    def handle_get_color(self, request, response):
        """Service: Turn on LED, sample color multiple times, average, turn off LED, return result."""
        self.get_logger().info("Service get_color called: turning LED on")
        
        # Turn on LED
        self.color_led_state = True
        if not self.send_command("v 1"):
            response.success = False
            response.message = "Failed to turn on LED (serial error)"
            return response
        
        time.sleep(0.2)  # Let LED stabilize

        # Collect multiple samples
        samples = []
        for i in range(5):
            if self.send_command("v"):  # Request color reading
                start = time.time()
                # Wait up to 0.3s for a response
                while time.time() - start < 0.3:
                    line = self.read_serial_line()
                    if line:
                        vals = self._parse_color_line(line)
                        if vals:
                            samples.append(vals)
                            self.get_logger().debug(f"Sample {i+1}: R={vals[0]} G={vals[1]} B={vals[2]}")
                            break
                    time.sleep(0.02)
            time.sleep(0.05)  # Small delay between samples

        # Turn off LED
        self.send_command("v 0")
        self.color_led_state = False
        self.get_logger().info("LED turned off")

        # Calculate average and identify color
        if samples:
            # Average each channel
            avg_r = sum(s[0] for s in samples) / len(samples)
            avg_g = sum(s[1] for s in samples) / len(samples)
            avg_b = sum(s[2] for s in samples) / len(samples)
            
            # Identify color name from averaged RGB
            color_name = self._identify_color(int(avg_r), int(avg_g), int(avg_b))
            
            color_str = f"{color_name} | R:{int(avg_r)} G:{int(avg_g)} B:{int(avg_b)} (from {len(samples)} samples)"
            response.success = True
            response.message = color_str
            self.get_logger().info(f"Detected color: {color_str}")
        else:
            response.success = False
            response.message = "No valid color samples received from sensor"
            self.get_logger().warning("No valid color samples")
        
        return response

    def _parse_color_line(self, line):
        """Parse color sensor response line and return [R, G, B] or None"""
        s = line.strip()
        
        # Try multiple formats:
        # Format 1: "COLOR R G B" or "Color: R G B"
        # Format 2: "R G B" (three space-separated integers)
        # Format 3: "R:value G:value B:value"
        
        # Remove common prefixes
        s = re.sub(r'^(COLOR|Color)\s*:?\s*', '', s, flags=re.IGNORECASE)
        
        # Try to extract three integers with flexible separators
        match = re.search(r'(\d+)[^\d]+(\d+)[^\d]+(\d+)', s)
        if match:
            try:
                r = int(match.group(1))
                g = int(match.group(2))
                b = int(match.group(3))
                # Sanity check: reject values that are clearly not color sensor data
                if 0 <= r <= 65535 and 0 <= g <= 65535 and 0 <= b <= 65535:
                    return [r, g, b]
            except (ValueError, IndexError):
                pass
        return None

    def _identify_color(self, r, g, b):
        """Identify color name from RGB values (16-bit sensor, 0-65535 range)
        Optimized for low-light/low-gain sensors with values typically < 100
        """
        # Log raw values for debugging
        self.get_logger().debug(f"_identify_color: raw R={r}, G={g}, B={b}")
        
        # Check if we got any meaningful reading
        total = r + g + b
        if total < 3:  # Less than 1 per channel = sensor error
            self.get_logger().warning("Color sensor returned near-zero values - check sensor!")
            return "Error: No reading"
        
        # For low-value sensors (typical with 50MS integration, 4X gain),
        # use RELATIVE ratios instead of absolute brightness
        max_val = max(r, g, b)
        min_val = min(r, g, b)
        
        # Avoid division by zero
        if max_val == 0:
            return "Black"
        
        # Calculate normalized ratios (0-1 range relative to max reading)
        r_ratio = r / max_val
        g_ratio = g / max_val
        b_ratio = b / max_val
        
        # Calculate color difference (how far apart the channels are)
        color_diff = (max_val - min_val) / max_val
        
        self.get_logger().debug(
            f"Ratios: R={r_ratio:.2f} G={g_ratio:.2f} B={b_ratio:.2f} "
            f"diff={color_diff:.2f} total={total}"
        )
        
        # Very low total with low difference = Black
        if total < 15 and color_diff < 0.3:
            return "Black"
        
        # Low color difference = grayscale (all channels similar)
        if color_diff < 0.20:  # Increased from 0.15 to catch more balanced colors
            if total < 20:
                return "Black"
            elif total < 40:
                return "Dark Gray"
            elif total < 70:
                return "Gray"
            elif total < 100:
                return "Light Gray"
            else:
                return "White"
        
        # Identify dominant color based on which channel is highest
        dominant_threshold = 0.85  # Channel needs to be 85% or higher to be "dominant"
        
        # Red is dominant
        if r >= max_val * 0.95:  # R is clearly the highest
            if g_ratio > 0.6:  # Significant green component
                return "Orange" if g_ratio > 0.75 else "Red-Orange"
            elif b_ratio > 0.6:  # Significant blue component
                return "Magenta"
            else:
                return "Red"
        
        # Green is dominant
        elif g >= max_val * 0.95:
            if r_ratio > 0.6:  # Significant red component
                return "Yellow" if r_ratio > 0.8 else "Yellow-Green"
            elif b_ratio > 0.6:  # Significant blue component
                return "Cyan" if b_ratio > 0.8 else "Cyan-Green"
            else:
                return "Green"
        
        # Blue is dominant
        elif b >= max_val * 0.95:
            if r_ratio > 0.6:  # Significant red component
                return "Purple" if r_ratio > 0.75 else "Blue-Purple"
            elif g_ratio > 0.6:  # Significant green component
                return "Cyan-Blue" if g_ratio > 0.75 else "Blue"
            else:
                return "Blue"
        
        # Mixed colors (two channels high, one low)
        # Yellow: R and G both high, B low
        if r_ratio > 0.85 and g_ratio > 0.85 and b_ratio < 0.6:
            return "Yellow"
        
        # Cyan: G and B both high, R low
        if g_ratio > 0.85 and b_ratio > 0.85 and r_ratio < 0.6:
            return "Cyan"
        
        # Magenta: R and B both high, G low
        if r_ratio > 0.85 and b_ratio > 0.85 and g_ratio < 0.6:
            return "Magenta"
        
        # If we can't determine clearly, return the dominant color with qualifier
        if r == max_val:
            return "Reddish" if color_diff > 0.3 else "Light Red"
        elif g == max_val:
            return "Greenish" if color_diff > 0.3 else "Light Green"
        else:
            return "Bluish" if color_diff > 0.3 else "Light Blue"

    def read_color_sensor(self):
        """Read color sensor data periodically (only when there are active subscribers)"""
        if not self.serial or not self.serial.is_open:
            return
        
        # Only poll color sensor if someone is subscribed to the topics
        # This prevents unnecessary serial traffic and warning spam
        if (self.color_sensor_pub.get_subscription_count() == 0 and 
            self.color_sensor_raw_pub.get_subscription_count() == 0):
            return
        
        # Send color read command (v command reads color data)
        if self.send_command("v"):
            start = time.time()
            # Wait up to 0.5s for response
            while time.time() - start < 0.5:
                line = self.read_serial_line()
                if not line:
                    continue
                
                # Try to parse the line as color data
                values = self._parse_color_line(line)
                if values:
                    self.color_rgb = values
                    
                    # Publish normalized color data (0-1 range)
                    color_msg = ColorRGBA()
                    # Normalize to 0-1 range (assuming 16-bit values from TCS34725)
                    color_msg.r = float(values[0]) / 65535.0
                    color_msg.g = float(values[1]) / 65535.0
                    color_msg.b = float(values[2]) / 65535.0
                    color_msg.a = 1.0
                    
                    self.color_sensor_pub.publish(color_msg)
                    
                    # Publish raw debug data
                    raw_msg = String()
                    raw_msg.data = f"R:{values[0]} G:{values[1]} B:{values[2]}"
                    self.color_sensor_raw_pub.publish(raw_msg)
                    
                    # Warn if values are suspiciously low (throttled to once every 10 seconds)
                    if max(values) < 100:
                        now = time.time()
                        if now - self._last_color_sensor_warn > 10.0:
                            self.get_logger().warning(
                                f"Color values very low: R={values[0]} G={values[1]} B={values[2]}. "
                                "Check sensor wiring, power, or Arduino 'v' command response"
                            )
                            self._last_color_sensor_warn = now
                    
                    self.get_logger().debug(f"Color RGB: {values}")
                    break
                else:
                    # Log unrecognized lines at debug level
                    self.get_logger().debug(f"Ignoring non-color line: '{line}'")
                    continue

    def update_sensor_data(self):
        """Periodically read sensor data from Arduino"""
        if not self.serial or not self.serial.is_open:
            return

        # Read encoders - robustly wait for a valid 4-int response, ignore other lines
        if self.send_command("e"):
            start = time.time()
            got_valid = False
            # Wait up to 1.0s for a valid encoder line (increased timeout)
            while time.time() - start < 10:
                line = self.read_serial_line()
                if not line:
                    continue
                
                # Check if this line is IMU data (9 space-separated floats) and handle it
                if len(line.split()) == 9:
                    try:
                        # Quick test: can we parse as 9 floats?
                        test_vals = list(map(float, line.split()))
                        self._handle_imu_line(line)
                        continue  # Keep looking for encoder data
                    except (ValueError, AttributeError):
                        pass  # Not IMU data, continue processing
                
                try:
                    counts = list(map(int, line.split()))
                    if len(counts) == 4:
                        self.encoder_counts = counts

                        # Publish raw encoder data
                        encoder_msg = String()
                        encoder_msg.data = (
                            f"{counts[0]},{counts[1]},{counts[2]},{counts[3]}"
                        )
                        self.raw_encoder_pub.publish(encoder_msg)
                        # Publish joint states for visualization
                        try:
                            self.publish_joint_states()
                        except Exception as e:
                            self.get_logger().warning(
                                f"Failed to publish joint states: {e}"
                            )
                        got_valid = True
                        break
                except ValueError:
                    # Ignore unrelated debug lines (e.g., "OK", IMU prints). Log at debug level.
                    self.get_logger().debug(
                        f"Ignoring non-encoder serial line while waiting for encoders: '{line}'"
                    )
                    continue
            if not got_valid:
                # Don't warn repeatedly for expected missing responses; log at debug level instead
                now = time.time()
                if now - self._last_invalid_encoder_warn > 5.0:
                    self.get_logger().debug(
                        "No valid encoder response received within timeout (debug)"
                    )
                    self._last_invalid_encoder_warn = now

        # IMU reads disabled in encoders-only mode
        # # Read IMU angle (only when moving based on Arduino state machine)
        # if self.robot_state == 1:  # MOVING state
        #     if self.send_command("i"):
        #         line = self.read_serial_line()
        #         if line:
        #             try:
        #                 self.imu_angle = float(line)
        #                 self.publish_imu_data()
        #             except ValueError:
        #                 pass

        # Ultrasonic reads disabled in encoders-only mode
        # # Read ultrasonic sensors
        # if self.robot_state != 0:  # Not OFFLOADING state
        #     if self.send_command("y"):
        #         line = self.read_serial_line()
        #         if line:
        #             try:
        #                 distances = list(map(float, line.split()))
        #                 if len(distances) == 2:
        #                     self.ultrasonic_ranges = [
        #                         d / 100.0 for d in distances
        #                     ]  # cm to meters
        #                     self.publish_ultrasonic_data()
        #             except ValueError:
        #                 pass

        # Read robot state
        if self.send_command("y"):
            line = self.read_serial_line()
            if line:
                try:
                    self.robot_state = int(line)
                    
                    # Check if robot is in offloading state (stepper active)
                    if self.robot_state == 0:  # OFFLOADING state
                        self.stepper_active = True
                    else:
                        self.stepper_active = False
                    
                    # Publish stepper status
                    status_msg = Bool()
                    status_msg.data = self.stepper_active
                    self.stepper_status_pub.publish(status_msg)
                    
                except ValueError:
                    pass

    def update_odometry(self):
        """Calculate and publish odometry from encoder data"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_update).nanoseconds / 1e9

        if dt <= 0:
            return

        # Define a threshold for the minimum change in encoder counts
        # INCREASED to 5 to prevent encoder noise from being integrated as motion
        encoder_threshold = 5  # Higher threshold reduces drift when stationary

        # Calculate wheel displacements in meters
        ticks_to_meters = (2 * math.pi * self.wheel_radius) / self.encoder_ticks_per_rev
        delta_ticks = [
            self.encoder_counts[i] - self.last_encoder_counts[i] for i in range(4)
        ]

        # Check if the change in encoder counts is below the threshold
        if all(abs(tick) < encoder_threshold for tick in delta_ticks):
            # Robot is stationary - CRITICAL: publish odometry with ZERO velocity
            # This anchors the EKF and prevents IMU drift from accumulating
            self.publish_odometry(0.0, 0.0, current_time)
            if self.publish_tf_enabled:
                self.publish_tf(current_time)
            self.last_odom_update = current_time
            # IMPORTANT: Don't update last_encoder_counts - keeps threshold check consistent
            return

        # Calculate wheel displacements in meters
        # Convert encoder ticks to distance
        ticks_to_meters = (2 * math.pi * self.wheel_radius) / self.encoder_ticks_per_rev

        delta_ticks = [
            self.encoder_counts[i] - self.last_encoder_counts[i] for i in range(4)
        ]

        # Average left and right wheel displacements
        # Assuming M1 and M3 are left wheels, M2 and M4 are right wheels
        left_distance = (delta_ticks[0] + delta_ticks[2]) / 2 * ticks_to_meters
        right_distance = (delta_ticks[1] + delta_ticks[3]) / 2 * ticks_to_meters

        # Calculate linear and angular displacement
        linear_displacement = (left_distance + right_distance) / 2
        angular_displacement = (right_distance - left_distance) / self.base_width

        # Update pose (ONLY for raw /odom visualization - NOT used by EKF!)
        # CRITICAL FIX: The EKF ignores our position/orientation and only uses velocities
        # This prevents double-integration conflicts between encoder theta and IMU yaw
        # The EKF will integrate our velocities with IMU orientation to get true pose
        self.odom_x += linear_displacement * math.cos(self.odom_theta)
        self.odom_y += linear_displacement * math.sin(self.odom_theta)
        self.odom_theta += angular_displacement

        # Calculate velocities (THESE are what the EKF uses for integration)
        linear_velocity = linear_displacement / dt
        angular_velocity = angular_displacement / dt

        # Publish odometry
        self.publish_odometry(linear_velocity, angular_velocity, current_time)

        # Publish TF (only if not using EKF - EKF will publish TF when enabled)
        if self.publish_tf_enabled:
            self.publish_tf(current_time)

        # Update last values
        self.last_encoder_counts = self.encoder_counts.copy()
        self.last_odom_update = current_time

    def publish_odometry(self, linear_vel, angular_vel, timestamp):
        """Publish odometry message with covariance for EKF sensor fusion"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set position
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        # CRITICAL: Normalize theta to [-pi, pi] to prevent drift
        self.odom_theta = math.atan2(math.sin(self.odom_theta), math.cos(self.odom_theta))

        # Convert theta to quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.odom_theta / 2)
        q.w = math.cos(self.odom_theta / 2)
        odom_msg.pose.pose.orientation = q

        # Set pose covariance (6x6 matrix, row-major)
        # [x, y, z, rotation about X, rotation about Y, rotation about Z]
        # For skid-steer: encoders good for position, BAD for rotation (wheels slip!)
        odom_msg.pose.covariance = [
            0.01,  0.0,   0.0,  0.0,  0.0,  0.0,   # x variance (encoders good)
            0.0,   0.01,  0.0,  0.0,  0.0,  0.0,   # y variance (encoders good)
            0.0,   0.0,   0.01, 0.0,  0.0,  0.0,   # z variance (not used in 2D)
            0.0,   0.0,   0.0,  0.01, 0.0,  0.0,   # roll variance (not used in 2D)
            0.0,   0.0,   0.0,  0.0,  0.01, 0.0,   # pitch variance (not used in 2D)
            0.0,   0.0,   0.0,  0.0,  0.0,  0.5    # yaw variance (HIGH - wheels slip!)
        ]

        # Set velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        # Set twist covariance (6x6 matrix, row-major)
        # [vx, vy, vz, rotation rate about X, rotation rate about Y, rotation rate about Z]
        odom_msg.twist.covariance = [
            0.01,  0.0,   0.0,  0.0,  0.0,  0.0,   # vx variance (encoders good)
            0.0,   0.01,  0.0,  0.0,  0.0,  0.0,   # vy variance (encoders good)
            0.0,   0.0,   0.01, 0.0,  0.0,  0.0,   # vz variance (not used)
            0.0,   0.0,   0.0,  0.01, 0.0,  0.0,   # vroll variance (not used)
            0.0,   0.0,   0.0,  0.0,  0.01, 0.0,   # vpitch variance (not used)
            0.0,   0.0,   0.0,  0.0,  0.0,  0.1    # vyaw variance (medium - wheels slip)
        ]

        self.odom_pub.publish(odom_msg)

    def publish_tf(self, timestamp):
        """Publish transform from odom to base_link"""
        # CRITICAL: This TF tells ROS where the robot is in the odom frame
        # SLAM/mapping uses this to keep the map stationary while robot moves
        
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame  # Parent: 'odom' (fixed reference)
        transform.child_frame_id = self.base_frame   # Child: 'base_link' (moves with robot)

        # Position in odom frame
        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.translation.z = 0.0

        # CRITICAL: Normalize quaternion to prevent numerical issues
        half_theta = self.odom_theta / 2.0
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_theta)
        q.w = math.cos(half_theta)
        
        # Normalize quaternion (ensure magnitude = 1)
        mag = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if mag > 1e-6:
            q.x /= mag
            q.y /= mag
            q.z /= mag
            q.w /= mag
        
        transform.transform.rotation = q

        self.tf_broadcaster.sendTransform(transform)

    def publish_imu_data(self):
        """Publish IMU data"""
        # IMU publishing disabled in encoders-only mode
        # imu_msg = Imu()
        # imu_msg.header.stamp = self.get_clock().now().to_msg()
        # imu_msg.header.frame_id = "imu_link"
        # q = Quaternion()
        # q.x = 0.0
        # q.y = 0.0
        # q.z = math.sin(self.imu_angle / 2)
        # q.w = math.cos(self.imu_angle / 2)
        # imu_msg.orientation = q
        # self.imu_pub.publish(imu_msg)

    def publish_ultrasonic_data(self):
        """Publish ultrasonic sensor data"""
        # Ultrasonic publishing disabled in encoders-only mode
        # left_range = Range()
        # left_range.header.stamp = self.get_clock().now().to_msg()
        # left_range.header.frame_id = "ultrasonic_left_link"
        # left_range.radiation_type = Range.ULTRASOUND
        # left_range.field_of_view = 0.26
        # left_range.min_range = 0.02
        # left_range.max_range = 4.0
        # left_range.range = self.ultrasonic_ranges[0]
        # self.ultrasonic_left_pub.publish(left_range)
        # right_range = Range()
        # right_range.header.stamp = self.get_clock().now().to_msg()
        # right_range.header.frame_id = "ultrasonic_right_link"
        # right_range.radiation_type = Range.ULTRASOUND
        # right_range.field_of_view = 0.26
        # right_range.min_range = 0.02
        # right_range.max_range = 4.0
        # right_range.range = self.ultrasonic_ranges[1]
        # self.ultrasonic_right_pub.publish(right_range)

    def publish_joint_states(self):
        """Publish wheel joint states for RViz visualization."""
        # Convert encoder counts to wheel angles (radians)
        # ticks -> revolutions -> radians
        ticks_to_rad = (2 * math.pi) / float(self.encoder_ticks_per_rev)

        # Assuming encoder_counts order: M1=front_left, M2=front_right, M3=rear_left, M4=rear_right
        fl = self.encoder_counts[0] * ticks_to_rad
        fr = self.encoder_counts[1] * ticks_to_rad
        rl = self.encoder_counts[2] * ticks_to_rad
        rr = self.encoder_counts[3] * ticks_to_rad

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            "front_left_wheel_joint",
            "back_left_wheel_joint",
            "front_right_wheel_joint",
            "back_right_wheel_joint",
        ]
        js.position = [fl, rl, fr, rr]

        self.joint_state_pub.publish(js)

    def publish_robot_state(self):
        """Publish robot state"""
        state_msg = Int32()
        state_msg.data = self.robot_state
        self.robot_state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROSArduinoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
