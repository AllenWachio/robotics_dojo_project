#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String, ColorRGBA
import serial
import threading
import time
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import select


class ROSArduinoBridge(Node):
    def __init__(self):
        super().__init__("ros_arduino_bridge")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("base_width", 0.3)  # Distance between left/right wheels
        self.declare_parameter("wheel_radius", 0.075)  # Wheel radius in meters
        self.declare_parameter(
            "encoder_ticks_per_rev", 44
        )  # Encoder ticks per revolution

        # Max speeds for mapping to PWM
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

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

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        # IMU and ultrasonic publishers temporarily disabled (encoders-only mode)
        # self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        # self.ultrasonic_left_pub = self.create_publisher(Range, "ultrasonic/left", 10)
        # self.ultrasonic_right_pub = self.create_publisher(Range, "ultrasonic/right", 10)
        self.robot_state_pub = self.create_publisher(Int32, "robot_state", 10)
        self.raw_encoder_pub = self.create_publisher(String, "raw_encoders", 10)
        # Joint states for visualization (RViz / joint_state_publisher expects /joint_states)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.data_timer = self.create_timer(0.05, self.update_sensor_data)  # 20Hz
        self.odom_timer = self.create_timer(0.1, self.update_odometry)  # 10Hz
        self.state_timer = self.create_timer(1.0, self.publish_robot_state)  # 1Hz

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

    def send_command(self, command):
        """Send command to Arduino with thread safety"""
        with self.serial_lock:
            if self.serial and self.serial.is_open:
                try:
                    full_command = command + "\r"
                    self.serial.write(full_command.encode())
                    self.serial.flush()
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

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor speeds for 4WD"""
        # Convert linear and angular velocities to wheel speeds
        linear = msg.linear.x
        angular = msg.angular.z

        # For 4-wheel drive skid-steer robot:
        # Left wheels: linear - (angular * base_width/2)
        # Right wheels: linear + (angular * base_width/2)
        # Handle in-place turns: if linear is near zero, map angular to
        # opposing wheel velocities so the robot rotates about its center.
        if abs(linear) < 1e-6 and abs(angular) > 1e-6:
            left_speed = -angular * (self.base_width / 2.0)
            right_speed = angular * (self.base_width / 2.0)
        else:
            left_speed = linear - (angular * self.base_width / 2.0)
            right_speed = linear + (angular * self.base_width / 2.0)

        # Convert m/s to PWM values using configured max_linear_speed
        # Protect against division by zero
        max_lin = float(self.max_linear_speed) if self.max_linear_speed != 0 else 0.5
        left_pwm = int((left_speed / max_lin) * 255)
        right_pwm = int((right_speed / max_lin) * 255)

        # Limit PWM values
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        # Send command to Arduino: m left1:right1:left2:right2
        # Assuming M1=front_left, M2=front_right, M3=rear_left, M4=rear_right
        command = f"m {left_pwm}:{right_pwm}:{left_pwm}:{right_pwm}"
        self.send_command(command)

        # Helpful debug: show computed speeds and PWM mapping so teleop can
        # be tuned if turning doesn't behave as expected.
        self.get_logger().debug(
            f"cmd_vel -> linear={linear:.3f} angular={angular:.3f} | "
            f"left_speed={left_speed:.3f} right_speed={right_speed:.3f} | "
            f"left_pwm={left_pwm} right_pwm={right_pwm} | cmd={command}"
        )

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
        if self.send_command("s"):
            line = self.read_serial_line()
            if line:
                try:
                    self.robot_state = int(line)
                except ValueError:
                    pass

    def update_odometry(self):
        """Calculate and publish odometry from encoder data"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_update).nanoseconds / 1e9

        if dt <= 0:
            return

        # Define a threshold for the minimum change in encoder counts
        encoder_threshold = 3  # Start with a threshold of 2 ticks

        # Calculate wheel displacements in meters
        ticks_to_meters = (2 * math.pi * self.wheel_radius) / self.encoder_ticks_per_rev
        delta_ticks = [
            self.encoder_counts[i] - self.last_encoder_counts[i] for i in range(4)
        ]

        # Check if the change in encoder counts is below the threshold
        if all(abs(tick) < encoder_threshold for tick in delta_ticks):
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

        # Update pose
        self.odom_x += linear_displacement * math.cos(self.odom_theta)
        self.odom_y += linear_displacement * math.sin(self.odom_theta)
        self.odom_theta += angular_displacement

        # Calculate velocities
        linear_velocity = linear_displacement / dt
        angular_velocity = angular_displacement / dt

        # Publish odometry
        self.publish_odometry(linear_velocity, angular_velocity, current_time)

        # Publish TF
        self.publish_tf(current_time)

        # Update last values
        self.last_encoder_counts = self.encoder_counts.copy()
        self.last_odom_update = current_time

    def publish_odometry(self, linear_vel, angular_vel, timestamp):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set position
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.odom_theta / 2)
        q.w = math.cos(self.odom_theta / 2)
        odom_msg.pose.pose.orientation = q

        # Set velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(odom_msg)

    def publish_tf(self, timestamp):
        """Publish transform from odom to base_link"""
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.translation.z = 0.0

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.odom_theta / 2)
        q.w = math.cos(self.odom_theta / 2)
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
