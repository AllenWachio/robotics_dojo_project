# New Sensors and Actuators Implementation

## Summary

Added support for:

1. **Color Sensor (TCS34725)** - RGB color detection
2. **Camera Servo** - Pan/tilt camera positioning
3. **Tipper Servo** - Tipping mechanism control
4. **Stepper Motor** - Conveyor belt for offloading objects

## Changes Made to `ros_arduino_bridge.py`

### New State Variables

```python
# Servo state
self.camera_servo_angle = 50      # Initial position from Arduino
self.tipper_servo_angle = 170     # Initial position from Arduino

# Color sensor state
self.color_rgb = [0, 0, 0]        # R, G, B raw values
self.color_led_state = False

# Stepper state
self.stepper_active = False
self.stepper_last_command = ""
```

### New ROS2 Topics

#### Publishers (Output - what you can monitor)

| Topic                 | Type      | Rate   | Description                         |
| --------------------- | --------- | ------ | ----------------------------------- |
| `/color_sensor/rgb`   | ColorRGBA | 5Hz    | Normalized RGB values (0-1 range)   |
| `/color_sensor/raw`   | String    | 5Hz    | **DEBUG**: Raw RGB values (0-65535) |
| `/stepper/active`     | Bool      | 20Hz   | Stepper motor running status        |
| `/stepper/debug`      | String    | On cmd | **DEBUG**: Stepper command details  |
| `/camera_servo/angle` | Int32     | On cmd | Camera servo position feedback      |
| `/camera_servo/debug` | String    | On cmd | **DEBUG**: Camera servo commands    |
| `/tipper_servo/angle` | Int32     | On cmd | Tipper servo position feedback      |
| `/tipper_servo/debug` | String    | On cmd | **DEBUG**: Tipper servo commands    |

#### Subscribers (Input - what you can control)

| Topic                   | Type   | Description                                   |
| ----------------------- | ------ | --------------------------------------------- |
| `/camera_servo/command` | Int32  | Set camera servo angle (0-180°)               |
| `/tipper_servo/command` | Int32  | Set tipper servo angle (0-180°)               |
| `/stepper/command`      | String | Stepper command: "rpm:distance_mm:flag"       |
| `/color_sensor/led`     | Bool   | Control color sensor LED (true=ON, false=OFF) |

### New Timer

```python
self.color_timer = self.create_timer(0.2, self.read_color_sensor)  # 5Hz color reading
```

## Arduino Communication Protocol

### Commands Sent to Arduino

| Command | Format                  | Example       | Description                               |
| ------- | ----------------------- | ------------- | ----------------------------------------- |
| `s`     | `s <index> <angle>`     | `s 0 90`      | Set servo angle (0=camera, 1=tipper)      |
| `q`     | `q <rpm>:<dist>:<flag>` | `q -25:400:0` | Stepper command                           |
| `v`     | `v` or `v 0` or `v 1`   | `v 1`         | Read color sensor / control LED           |
| `y`     | `y`                     | `y`           | Get robot state (includes stepper status) |

### Expected Responses from Arduino

| Command    | Response Format           | Example                                          |
| ---------- | ------------------------- | ------------------------------------------------ |
| Color read | `R G B` (space-separated) | `45231 12456 8932`                               |
| State      | Single integer            | `0` (OFFLOADING), `1` (MOVING), `2` (STATIONARY) |

## Testing When Running `./01_arduino_only.sh`

### ✅ YES - The new code WILL run!

When you run `./01_arduino_only.sh`, it launches:

1. **Robot State Publisher** - TF tree
2. **Arduino Bridge Node** - Your updated `ros_arduino_bridge.py`

All new sensors and actuators will be available!

### Debug Topics (Like `/raw_encoders`)

These topics are perfect for debugging, similar to `/raw_encoders`:

```bash
# Monitor all debug topics at once
ros2 topic list | grep -E "(raw|debug)"

# Expected output:
# /raw_encoders
# /color_sensor/raw
# /stepper/debug
# /camera_servo/debug
# /tipper_servo/debug
```

### Testing Commands

#### 1. **Camera Servo Test**

```bash
# Set camera to 90 degrees (center)
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 90"

# Monitor feedback
ros2 topic echo /camera_servo/angle

# Monitor debug output
ros2 topic echo /camera_servo/debug
```

#### 2. **Tipper Servo Test**

```bash
# Tip to 45 degrees
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 45"

# Return to initial position
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 170"

# Monitor
ros2 topic echo /tipper_servo/debug
```

#### 3. **Stepper Motor Test**

```bash
# Move forward 400mm at 25 RPM
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '25:400:0'"

# Return to zero position
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '0:0:1'"

# Monitor status
ros2 topic echo /stepper/active

# Monitor debug
ros2 topic echo /stepper/debug
```

#### 4. **Color Sensor Test**

```bash
# Turn LED ON
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"

# Turn LED OFF
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: false"

# Monitor raw RGB values (like /raw_encoders)
ros2 topic echo /color_sensor/raw

# Monitor normalized values
ros2 topic echo /color_sensor/rgb
```

### Quick Debug Script

Create this script for easy testing: `~/test_new_sensors.sh`

```bash
#!/bin/bash
echo "=== New Sensors Debug Monitor ==="
echo ""
echo "Opening debug topics in separate terminals..."

# Open each debug topic in a new terminal
gnome-terminal -- bash -c "ros2 topic echo /camera_servo/debug; exec bash" &
gnome-terminal -- bash -c "ros2 topic echo /tipper_servo/debug; exec bash" &
gnome-terminal -- bash -c "ros2 topic echo /stepper/debug; exec bash" &
gnome-terminal -- bash -c "ros2 topic echo /color_sensor/raw; exec bash" &

echo "Debug terminals opened!"
echo ""
echo "Available commands to test:"
echo "  Camera:  ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 'data: 90'"
echo "  Tipper:  ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 'data: 45'"
echo "  Stepper: ros2 topic pub --once /stepper/command std_msgs/msg/String \"data: '25:400:0'\""
echo "  Color:   ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool 'data: true'"
```

### Monitor All Topics

```bash
# List all topics (should see new ones)
ros2 topic list

# Monitor robot state (shows if stepper is active)
ros2 topic echo /robot_state

# Monitor stepper status
ros2 topic echo /stepper/active

# Check all topic rates
ros2 topic hz /color_sensor/rgb
ros2 topic hz /color_sensor/raw
```

### Example Debug Output

When you publish a camera servo command:

```bash
$ ros2 topic echo /camera_servo/debug
data: "Camera servo commanded to 90°, sent: 's 0 90'"
---
```

When stepper is running:

```bash
$ ros2 topic echo /stepper/debug
data: "Stepper: rpm=25, dist=400mm, flag=0, cmd='q 25:400:0'"
---
```

When color sensor reads:

```bash
$ ros2 topic echo /color_sensor/raw
data: "R:45231 G:12456 B:8932"
---
```

## Integration with Robot State Machine

The Arduino state machine now controls sensor data transmission:

- **STATE_OFFLOADING (0)**: Stepper active, minimal data
- **STATE_MOVING (1)**: All sensors active
- **STATE_STATIONARY (2)**: Limited data

The Python bridge automatically:

- Detects state from Arduino
- Updates `/stepper/active` accordingly
- Maintains all debug topics regardless of state

## Rebuild Instructions

After code changes:

```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

Then test:

```bash
./01_arduino_only.sh
```

## Troubleshooting

### No response from servos?

```bash
# Check if commands are being sent
ros2 topic echo /camera_servo/debug

# Check Arduino connection
ros2 topic echo /raw_encoders  # Should see encoder data
```

### Color sensor not reading?

```bash
# Check raw topic
ros2 topic echo /color_sensor/raw

# Turn on LED first
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"
```

### Stepper not moving?

```bash
# Check status
ros2 topic echo /stepper/active

# Check debug output
ros2 topic echo /stepper/debug

# Verify robot state
ros2 topic echo /robot_state  # Should be 0 when offloading
```

## Next Steps

1. **Rebuild the package** (see above)
2. **Run `./01_arduino_only.sh`**
3. **Test each sensor individually** using the commands above
4. **Monitor debug topics** to verify communication
5. **Integrate into behavior trees** for autonomous operation

All debug topics work exactly like `/raw_encoders` - they show the raw data being sent/received for easy debugging!
