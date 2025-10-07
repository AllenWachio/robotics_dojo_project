# Terminal Testing Commands for New Sensors/Actuators

## Quick Test All Debug Topics (Monitor First)

Open separate terminals to monitor debug output:

```bash
# Terminal 1: Monitor camera servo debug
ros2 topic echo /camera_servo/debug

# Terminal 2: Monitor tipper servo debug
ros2 topic echo /tipper_servo/debug

# Terminal 3: Monitor stepper debug
ros2 topic echo /stepper/debug

# Terminal 4: Monitor color sensor raw data
ros2 topic echo /color_sensor/raw

# Terminal 5: Monitor robot state
ros2 topic echo /robot_state
```

---

## 1. Camera Servo Tests

### Test Camera Movement

```bash
# Move camera to center (90°)
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 90"

# Move camera left (0°)
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 0"

# Move camera right (180°)
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 180"

# Move camera up/tilt (adjust based on your mount, e.g., 120°)
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 120"

# Return to default position (50°)
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 50"
```

### Monitor Camera Feedback

```bash
# Check current angle
ros2 topic echo /camera_servo/angle --once

# Watch angle changes live
ros2 topic echo /camera_servo/angle
```

---

## 2. Tipper Servo Tests

### Test Tipper Movement

```bash
# Tip down (45°) - dumps object
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 45"

# Tip horizontal (90°)
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 90"

# Return to upright position (170°)
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 170"

# Test different angles
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 100"
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 130"
```

### Monitor Tipper Feedback

```bash
# Check current angle
ros2 topic echo /tipper_servo/angle --once

# Watch angle changes live
ros2 topic echo /tipper_servo/angle
```

---

## 3. Stepper Motor (Conveyor) Tests

### Test Stepper Movement

```bash
# Move forward 100mm at 25 RPM
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '25:100:0'"

# Move forward 400mm at 25 RPM (full offload)
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '25:400:0'"

# Move backward 200mm at 15 RPM (slower)
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '-15:200:0'"

# Return to zero position (home)
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '0:0:1'"

# Fast movement: 35 RPM for 300mm
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '35:300:0'"
```

### Monitor Stepper Status

```bash
# Check if stepper is active (true/false)
ros2 topic echo /stepper/active

# Watch debug output
ros2 topic echo /stepper/debug

# Check robot state (0=OFFLOADING when stepper active)
ros2 topic echo /robot_state --once
```

---

## 4. Color Sensor Tests

### Control Color Sensor LED

```bash
# Turn LED ON (for better readings)
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"

# Turn LED OFF
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: false"
```

### Monitor Color Readings

```bash
# Watch raw RGB values (0-65535 range)
ros2 topic echo /color_sensor/raw

# Watch normalized RGB values (0-1 range)
ros2 topic echo /color_sensor/rgb

# Get one reading only
ros2 topic echo /color_sensor/raw --once
ros2 topic echo /color_sensor/rgb --once
```

### Test Color Detection

```bash
# 1. Turn LED on
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"

# 2. Place different colored objects under sensor and monitor:
ros2 topic echo /color_sensor/raw

# You should see different RGB values for:
# - Red object: High R, low G, low B
# - Green object: Low R, high G, low B
# - Blue object: Low R, low G, high B
# - White object: High R, high G, high B
# - Black object: Low R, low G, low B
```

---

## 5. Combined Workflow Tests

### Full Offload Sequence Test

```bash
# 1. Monitor stepper status in one terminal
ros2 topic echo /stepper/active

# 2. Position tipper upright
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 170"

# 3. Run conveyor forward 400mm
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '25:400:0'"

# 4. Wait for stepper to finish (stepper/active becomes false)
# Then tip to dump
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 45"

# 5. Return tipper upright
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 170"

# 6. Return conveyor to zero
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '0:0:1'"
```

### Color Detection + Camera Positioning

```bash
# 1. Turn on color sensor LED
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"

# 2. Position camera to look at object
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 90"

# 3. Monitor color readings
ros2 topic echo /color_sensor/raw
```

---

## 6. Quick Diagnostic Commands

### Check All Topic Rates

```bash
# Check if color sensor is publishing at ~5Hz
ros2 topic hz /color_sensor/raw

# Check stepper status update rate
ros2 topic hz /stepper/active

# Check servo feedback (only updates on command)
ros2 topic hz /camera_servo/angle
```

### List All Available Topics

```bash
ros2 topic list
```

### Check Topic Types

```bash
ros2 topic type /camera_servo/command
ros2 topic type /stepper/command
ros2 topic type /color_sensor/rgb
```

### Echo Multiple Topics Simultaneously

```bash
# Open in multiple terminals
ros2 topic echo /camera_servo/debug &
ros2 topic echo /tipper_servo/debug &
ros2 topic echo /stepper/debug &
ros2 topic echo /color_sensor/raw &
```

---

## 7. Verify Communication with Arduino

### Check if Commands Reach Arduino

```bash
# Monitor debug topics while sending commands
# Terminal 1:
ros2 topic echo /camera_servo/debug

# Terminal 2:
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 90"

# You should see: "Camera servo commanded to 90°, sent: 's 0 90'"
```

### Check Raw Encoder Data (Verify Arduino Connection)

```bash
# If this works, Arduino is connected properly
ros2 topic echo /raw_encoders --once

# Should show something like: "data: '123,456,789,012'"
```

---

## 8. Simple Test Script

Create a file `~/quick_test.sh`:

```bash
#!/bin/bash
echo "=== Quick Sensor Test ==="
echo ""

echo "1. Testing Camera Servo..."
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 90"
sleep 2
echo "   Camera angle:"
ros2 topic echo /camera_servo/angle --once
echo ""

echo "2. Testing Tipper Servo..."
ros2 topic pub --once /tipper_servo/command std_msgs/msg/Int32 "data: 90"
sleep 2
echo "   Tipper angle:"
ros2 topic echo /tipper_servo/angle --once
echo ""

echo "3. Testing Color Sensor LED..."
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"
sleep 1
echo "   Color reading:"
ros2 topic echo /color_sensor/raw --once
echo ""

echo "4. Testing Stepper (small movement)..."
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '25:50:0'"
sleep 2
echo "   Stepper active:"
ros2 topic echo /stepper/active --once
echo ""

echo "=== Test Complete ==="
```

Make it executable and run:

```bash
chmod +x ~/quick_test.sh
~/quick_test.sh
```

---

## Expected Behavior

### Camera Servo (s 0 <angle>)

- Should hear servo move
- Check `/camera_servo/angle` feedback matches command
- `/camera_servo/debug` shows command sent to Arduino

### Tipper Servo (s 1 <angle>)

- Should hear servo move
- Check `/tipper_servo/angle` feedback matches command
- `/tipper_servo/debug` shows command sent to Arduino

### Stepper Motor (q rpm:distance:flag)

- Should hear stepper running
- `/stepper/active` = true while moving
- `/robot_state` = 0 (OFFLOADING) during movement
- `/stepper/debug` shows parsed command

### Color Sensor (v or v 0/1)

- `/color_sensor/raw` updates every ~0.2s (5Hz)
- RGB values change when different colors placed under sensor
- LED control visible on sensor board

---

## Troubleshooting

### No servo movement?

```bash
# Check if command is being sent
ros2 topic echo /camera_servo/debug --once

# Manually send again
ros2 topic pub --once /camera_servo/command std_msgs/msg/Int32 "data: 90"

# Check Arduino connection
ros2 topic echo /raw_encoders --once
```

### No color readings?

```bash
# Turn LED on first
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"

# Check if raw topic is publishing
ros2 topic hz /color_sensor/raw

# Force a reading
ros2 topic echo /color_sensor/raw --once
```

### Stepper not moving?

```bash
# Check if command was received
ros2 topic echo /stepper/debug

# Check current status
ros2 topic echo /stepper/active --once

# Try smaller distance first
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '15:50:0'"
```

---

## Summary of Command Formats

| Device       | Command Format                              | Example            |
| ------------ | ------------------------------------------- | ------------------ |
| Camera Servo | `std_msgs/msg/Int32 "data: <angle>"`        | `data: 90`         |
| Tipper Servo | `std_msgs/msg/Int32 "data: <angle>"`        | `data: 45`         |
| Stepper      | `std_msgs/msg/String "data: 'rpm:mm:flag'"` | `data: '25:400:0'` |
| Color LED    | `std_msgs/msg/Bool "data: <true/false>"`    | `data: true`       |

All commands use `ros2 topic pub --once <topic> <type> <data>`
