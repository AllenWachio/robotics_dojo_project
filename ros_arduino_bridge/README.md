# ros_arduino_bridge

A small ROS 2 ament_python package that bridges an Arduino-based 4WD robot to ROS 2.

This README documents the serial protocol expected by the node, build and launch steps, quick checks, and troubleshooting tips.

## Serial protocol (Arduino <-> ROS node)

The ROS node communicates with the Arduino using a simple ASCII command/response protocol over a serial port. Each command sent from ROS expects a single-line response terminated by a newline. Commands are sent with a trailing carriage return ("\r").

Commands sent by ROS (requests):

- r : Reset encoders (no response required but Arduino may respond with an acknowledgment).
- e : Request encoders. Arduino should reply with four integers separated by spaces, e.g.:
  "123 456 789 1011\n" (M1 M2 M3 M4)
- i : Request IMU angle (only requested when robot_state == 1). Arduino should reply with one float (radians), e.g.:
  "1.5708\n"
- y : Request ultrasonic distances. Arduino should reply with two numbers in centimeters, e.g.:
  "120.5 118.0\n" (left_cm right_cm)
- s : Request robot state. Arduino should reply with a single integer state code (e.g., 0,1,2).

Motor command format (from ROS to Arduino):

- The node sends a motor command string of the form:
  m left1:right1:left2:right2\r
  where left*/right* are integer PWM values in [-255, 255]. Example:
  "m 120:120:120:120\r"
  Mapping from Twist -> PWM is done in the node assuming a max linear speed (default mapping uses 0.5 m/s -> 255 PWM). Calibrate this mapping for your robot.

Important implementation notes

- The Arduino must respond promptly with one line per request. The node uses a small select() timeout on the serial port to read lines.
- Encoder rollover is not handled by the node; if your Arduino encoder counters wrap, consider sending delta counts or using larger counters and handling wrap in the node.
- The node expects the motor ordering M1=front_left, M2=front_right, M3=rear_left, M4=rear_right by default.

## Build & install

From the root of your ROS 2 workspace (the directory that contains `src/`):

```bash
# build only this package
colcon build --packages-select ros_arduino_bridge

# source the workspace (bash)
source install/setup.bash
```

Dependencies

- System / apt: python3-pyserial (or install using pip)
- Python (package): pyserial is added to `install_requires` so `pip`-based installs will pull it in.

If you prefer pip+venv you can also install the package's Python deps with:

```bash
python3 -m pip install -r <(python3 -c "import setuptools; print('pyserial')")
```

## Run / Launch

Default launch (starts robot_state_publisher, the bridge node, and RViz):

```bash
ros2 launch ros_arduino_bridge arduino_bridge.py
```

Common overrides:

- Disable RViz:

```bash
ros2 launch ros_arduino_bridge arduino_bridge.py use_rviz:=false
```

- Set serial port (e.g., if Arduino is /dev/ttyACM0):

```bash
ros2 launch ros_arduino_bridge arduino_bridge.py serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

Run node directly for debugging (after sourcing):

```bash
ros2 run ros_arduino_bridge ros_arduino_bridge
```

## Control with keyboard (teleop)

This package includes support for `teleop_twist_keyboard` (declared in `package.xml`). To control the robot using your keyboard:

1. Launch the bridge (and optionally RViz):

```bash
ros2 launch ros_arduino_bridge arduino_bridge.py
```

2. In another terminal (after sourcing), start teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. Focus the teleop terminal and press the forward key (usually the up-arrow or 'i' depending on the teleop layout) to publish `cmd_vel` messages. The bridge subscribes to `/cmd_vel` and converts Twist messages into motor PWM commands sent to the Arduino.

Note: By default, forward on teleop sends a positive linear.x which the node maps into PWM using the configured `max_speed` mapping; tune `base_width`, `wheel_radius`, and `encoder_ticks_per_rev` parameters if motion looks incorrect.

## RViz wheel visualization

The node now publishes `sensor_msgs/JointState` messages on `/joint_states` (wheel joints: `front_left_wheel_joint`, `back_left_wheel_joint`, `front_right_wheel_joint`, `back_right_wheel_joint`).
When RViz displays the robot model (loaded via `robot_state_publisher`) those joint positions will animate the wheels as encoder counts change.

Quick checks

- List topics:

```bash
ros2 topic list
```

- Echo raw encoders:

```bash
ros2 topic echo /raw_encoders
```

- Check odom:

```bash
ros2 topic echo /odom
```

Troubleshooting

- Serial port permission denied:
  - Add your user to the dialout group on Linux:

```bash
sudo usermod -a -G dialout $USER
# then log out and back in or run a new session
```

- If the node cannot connect to the serial port, ensure the `serial_port` parameter matches your device and the Arduino is powered / connected.
- If you see malformed data warnings, verify the Arduino prints numeric responses exactly as expected (single-line, space-separated numbers where required).
- If TF/odom drifts badly, verify `encoder_ticks_per_rev`, `wheel_radius`, and `base_width` parameters match the real robot.

Contributing

- Update the `urdf_loader` package_name argument if you split URDF into another package.
- Consider improving protocol reliability by adding response prefixes or sequence numbers.

License

- BSD (as declared in package.xml)
