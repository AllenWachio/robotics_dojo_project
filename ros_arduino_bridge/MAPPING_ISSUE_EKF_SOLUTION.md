# Mapping Issues and EKF Solution

## Problem: Choppy Mapping with Breaks on Turns

### Symptoms

- Map is very choppy during navigation
- Map breaks/corrupts when robot makes even minute turns
- Robot appears to "jump" in RViz during rotation

### Root Cause

**You are NOT running with EKF sensor fusion enabled!**

The wheel encoders provide poor rotation estimates on a 4WD skid-steer robot because:

1. **Wheel slippage during turns** - Inner/outer wheels slip differently
2. **No direct yaw measurement** - Encoders calculate rotation from wheel differences
3. **Accumulated error** - Small errors compound quickly during turns

## Solution: Enable EKF Sensor Fusion

### What EKF Does

The Extended Kalman Filter fuses:

- **Wheel encoders** → Good for linear velocity (forward/backward)
- **IMU gyroscope** → Excellent for rotation rate (yaw velocity)
- **IMU orientation** → Direct yaw measurement (corrects encoder drift)

Result: Smooth, accurate odometry even during aggressive turns!

## How to Enable EKF

### Step 1: Install robot_localization (if not already done)

```bash
# On the Raspberry Pi:
sudo apt update
sudo apt install ros-humble-robot-localization
```

### Step 2: Run with EKF flag

```bash
# Instead of:
./01_arduino_only.sh

# Use:
./01_arduino_only.sh --ekf
```

### Step 3: Verify EKF is Running

```bash
# Check that EKF node is running:
ros2 node list | grep ekf
# Should show: /ekf_filter_node

# Verify topics:
ros2 topic list | grep odom
# Should show:
#   /odom                    (raw encoder odometry)
#   /odometry/filtered       (EKF fused odometry)

# Check filtered odometry:
ros2 topic echo /odometry/filtered --once
```

### Step 4: Use Filtered Odometry for Mapping

When launching SLAM (slam_toolbox or similar), configure it to use `/odometry/filtered` instead of `/odom`.

## Expected Results

### Before EKF (current state)

- ❌ Map jitters during turns
- ❌ Robot "teleports" in RViz
- ❌ Rotation estimates jump around
- ❌ Map becomes unusable after a few turns

### After EKF (with --ekf flag)

- ✅ Smooth map updates during turns
- ✅ Robot moves fluidly in RViz
- ✅ Accurate rotation tracking
- ✅ Stable, high-quality maps

## Configuration Files

The EKF is already configured for your robot:

- `config/ekf_config.yaml` - EKF parameters
- `deployment/pi/launch/arduino_only.launch.py` - Launch with EKF support
- `deployment/scripts/pi/01_arduino_only.sh` - Script with --ekf flag

## Troubleshooting

### Check if EKF is running:

```bash
ros2 node info /ekf_filter_node
```

### Monitor raw vs filtered odometry:

```bash
# Terminal 1 (raw encoder odometry):
ros2 topic echo /odom

# Terminal 2 (filtered odometry):
ros2 topic echo /odometry/filtered
```

### Check TF tree:

```bash
ros2 run tf2_tools view_frames
# Should show: odom → base_link (published by EKF)
```

### If EKF won't start:

```bash
# Check for errors:
ros2 launch ros_arduino_bridge arduino_only.launch.py use_ekf:=true

# Verify IMU is publishing:
ros2 topic hz /imu/data
# Should be ~5 Hz

# Verify raw odometry:
ros2 topic hz /odom
# Should be ~20-30 Hz
```

## Key Points

1. **Always use --ekf flag for mapping/navigation**
2. The raw `/odom` topic will still exist (for debugging)
3. Use `/odometry/filtered` for mapping/navigation
4. EKF corrects wheel slippage automatically
5. Works best when robot is moving (needs sensor data)

## Performance Impact

- Minimal CPU usage (~1-2%)
- No noticeable delay
- Much better than trying to tune encoder-only odometry

## Next Steps

1. ✅ Install robot_localization (if needed)
2. ✅ Run `./01_arduino_only.sh --ekf`
3. ✅ Verify `/odometry/filtered` topic exists
4. ✅ Configure SLAM to use filtered odometry
5. ✅ Test mapping - should be smooth and stable!
