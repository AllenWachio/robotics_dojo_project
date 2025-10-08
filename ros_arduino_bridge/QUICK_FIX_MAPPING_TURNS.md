# Quick Fix Guide: Map Breaking During Turns

## TL;DR - The Problem

Your SLAM system is likely using **raw wheel encoder odometry** instead of **IMU-fused odometry**. When the robot turns, wheel encoders slip and provide incorrect rotation data, causing the map to break.

## The Solution (3 Steps)

### 1. Use the New Launch File

**Instead of:**

```bash
ros2 launch ros_arduino_bridge slam_launch.py
```

**Use this:**

```bash
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
```

This new launch file:

- ✅ Starts EKF sensor fusion
- ✅ Disables TF conflict
- ✅ Configures SLAM to use filtered odometry

---

### 2. Verify It's Working

Run the diagnostic script:

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/scripts
./diagnose_mapping_issue.sh
```

**You should see:**

- ✅ EKF node running
- ✅ /odometry/filtered topic exists
- ✅ SLAM subscribed to /odometry/filtered
- ✅ arduino_bridge publish_tf = False

---

### 3. Test Mapping

```bash
# Start the complete system
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py

# In another terminal: visualize
rviz2

# Configure RViz:
# - Fixed Frame: "map"
# - Add → Map → /map
# - Add → LaserScan → /scan
# - Add → TF
# - Add → Odometry → /odometry/filtered

# Drive the robot and make turns
# Map should stay consistent!
```

---

## What Changed?

### Before (Broken):

```
Arduino → /odom (raw encoders) → SLAM → ❌ Map breaks during turns
                                  (wheels slip!)
```

### After (Fixed):

```
Arduino → /odom (encoders) ↘
Arduino → /imu/data (IMU)   → EKF → /odometry/filtered → SLAM → ✅ Smooth mapping!
                                     (corrects wheel slip!)
```

---

## If It Still Doesn't Work

### Check 1: Is robot_localization installed?

```bash
sudo apt update
sudo apt install ros-humble-robot-localization
```

### Check 2: Is IMU sending data?

```bash
ros2 topic echo /imu/data --once
# Should show orientation, angular_velocity, linear_acceleration
# NOT all zeros!
```

### Check 3: Rebuild the package

```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

### Check 4: Check Arduino IMU code

Arduino must respond to command `i` with 9 values:

```
yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z
```

Example:

```
45.30 2.10 -1.50 0.05 -0.02 12.34 0.12 -0.05 9.78
```

---

## Parameter Tuning (If Needed)

### If turns are still problematic:

**Option A: Make SLAM more tolerant**

Edit `config/mapper_params_online_async.yaml`:

```yaml
minimum_travel_heading: 0.3 # Increase (fewer scans during turns)
angle_variance_penalty: 0.1 # Lower (trust odometry more)
```

**Option B: Trust IMU more**

Edit `config/ekf_config.yaml`:

```yaml
imu0_orientation_covariance: [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.02] # Lower = trust more
```

---

## Monitoring Commands

While system is running:

```bash
# Compare raw vs filtered odometry
ros2 topic echo /odom/pose/pose/orientation --once
ros2 topic echo /odometry/filtered/pose/pose/orientation --once

# Check publishing rates
ros2 topic hz /odometry/filtered  # Should be ~20Hz

# Monitor TF tree
ros2 run tf2_ros tf2_monitor

# Check for errors
ros2 topic echo /diagnostics
```

---

## Common Mistakes

❌ **Launching without EKF:**

```bash
# DON'T use the old launch file:
ros2 launch ros_arduino_bridge slam_launch.py  # ❌ No sensor fusion!
```

❌ **TF conflict:**

```bash
# Arduino bridge must NOT publish TF when using EKF
ros2 param set /ros_arduino_bridge publish_tf false
```

❌ **SLAM using wrong odometry:**

```bash
# SLAM must use /odometry/filtered, not /odom
# Check with:
ros2 topic info /odometry/filtered --verbose
# Should show slam_toolbox as subscriber
```

---

## Expected Behavior

### Before Fix:

- 🔴 Robot drives straight: map OK
- 🔴 Robot turns: map distorts/breaks
- 🔴 Scan misalignment during rotation
- 🔴 Position jumps after turns

### After Fix:

- ✅ Robot drives straight: map perfect
- ✅ Robot turns: map stays consistent
- ✅ Scans align during rotation
- ✅ Smooth odometry throughout

---

## Files to Review

1. **`MAPPING_TURN_ISSUE_FIX.md`** - Detailed analysis and solutions
2. **`launch/slam_with_sensor_fusion.launch.py`** - Integrated launch file
3. **`scripts/diagnose_mapping_issue.sh`** - Diagnostic tool
4. **`SENSOR_FUSION_SUMMARY.md`** - Sensor fusion background
5. **`EKF_IMPLEMENTATION_REVIEW.md`** - EKF configuration details

---

## Contact / Help

If you're still experiencing issues:

1. Run the diagnostic script and share output
2. Check logs: `ros2 launch ... --log-level DEBUG`
3. Record a bag file during problematic turn:
   ```bash
   ros2 bag record /odom /imu/data /odometry/filtered /scan
   ```

---

## Summary Checklist

Before asking for help, verify:

- [ ] Using new launch file: `slam_with_sensor_fusion.launch.py`
- [ ] `robot_localization` package installed
- [ ] `/odometry/filtered` topic publishing
- [ ] SLAM subscribed to `/odometry/filtered`
- [ ] IMU sending valid data (not zeros)
- [ ] No TF warnings in console
- [ ] Package rebuilt after changes
- [ ] Ran diagnostic script successfully

If all checked ✅ and still broken, it's likely an IMU calibration or SLAM parameter tuning issue.
