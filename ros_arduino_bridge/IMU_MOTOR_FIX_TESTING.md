# IMU Setup and Motor Control Fix - Testing Guide

## Changes Applied

### 1. IMU Improvements
- ✅ Changed IMU `frame_id` from `base_link` to `imu_link` (proper TF tree)
- ✅ Added IMU calibration on startup (removes gyro bias)
- ✅ Updated covariance values for MPU6050 DMP:
  - Orientation: 0.001 (DMP fusion is excellent)
  - Angular velocity: 0.02 (raw gyro has some drift)
  - Linear acceleration: 0.5 (very noisy on small robots)
- ✅ Added static transform publisher: `base_link → imu_link` in launch file
- ✅ Added `imu0_frame_id: imu_link` to EKF config

### 2. Motor Control Fix
- ✅ Fixed 4-wheel turning - all motors now receive proper commands
- ✅ Calculate PWM independently for each wheel (FL, FR, RL, RR)
- ✅ Added INFO-level logging for turn commands (easier debugging)
- ✅ Improved differential drive math for center rotation

### 3. LIDAR Map Rotation Fix
- ✅ Normalized theta angle in odometry (prevents wraparound drift)
- ✅ Normalized quaternions in TF publishing (prevents numerical errors)
- ✅ Added detailed comments about TF tree structure
- ✅ Created TF diagnostic script (`check_tf_tree.py`)

---

## Testing Procedure (On Raspberry Pi)

### Step 1: Build and Deploy
```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

### Step 2: Test IMU Calibration
```bash
# Launch bridge (will auto-calibrate IMU)
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false

# Expected output:
# [INFO] Starting IMU calibration - keep robot STATIONARY for 5 seconds...
# [INFO] IMU calibration complete (XX samples). Gyro bias: X=... Y=... Z=...
```

**IMPORTANT:** Keep robot completely stationary during the 5-second calibration!

### Step 3: Verify IMU Data
```bash
# Check IMU topic (should show frame_id: imu_link)
ros2 topic echo /imu/data --once

# Expected output:
# header:
#   frame_id: imu_link  # <-- MUST be 'imu_link', not 'base_link'
# orientation: ...
# angular_velocity: ...
# linear_acceleration: ...
```

### Step 4: Test Motor Control
```bash
# Test turn left (should see all 4 wheel PWM values)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Check terminal logs:
# [INFO] TURN: linear=0.000 angular=0.500 | FL=-31 FR=31 RL=-31 RR=31 | cmd='m -31:31:-31:31' success=True
```

**Verify:**
- All 4 PWM values are non-zero
- Left wheels (FL, RL) are negative
- Right wheels (FR, RR) are positive
- Back right wheel (RR) actually moves!

### Step 5: Test with Teleop
```bash
# Terminal 1: Bridge
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false

# Terminal 2: EKF fusion
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Terminal 3: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Test patterns:**
1. **Forward/backward:** All wheels should move same direction
2. **Turn left:** Left wheels reverse, right wheels forward
3. **Turn right:** Right wheels reverse, left wheels forward
4. **Spin in place:** Opposite wheel directions, robot rotates at center

### Step 6: Check TF Tree
```bash
# Option A: Use our diagnostic script
ros2 run ros_arduino_bridge check_tf_tree.py

# Expected output:
# ✓ map → odom: x=... y=... θ=...
# ✓ odom → base_link: x=... y=... θ=...
# ✓ base_link → laser: x=... y=... θ=...
# ✓ base_link → imu_link: x=0.000 y=0.000 θ=0.000

# Option B: Generate TF tree PDF
ros2 run tf2_tools view_frames
# Opens frames.pdf showing full TF tree
```

**Expected TF structure:**
```
map → odom → base_link → laser
                       → imu_link
```

### Step 7: Test SLAM with Fixed Map
```bash
# Terminal 1: Bridge + EKF
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Terminal 2: SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: RViz
rviz2
```

**In RViz:**
1. **Set Fixed Frame to `map`** (top left dropdown) - CRITICAL!
2. Add displays:
   - TF (show all frames)
   - LaserScan (topic: `/scan`)
   - Odometry (topic: `/odom`)
   - Odometry (topic: `/odometry/filtered`)
   - Map (topic: `/map`)

3. **Test map stability:**
   - Drive forward: map points should stay fixed
   - **Rotate in place:** Scan points should NOT rotate with robot!
   - If map rotates with robot, check Fixed Frame setting

---

## Troubleshooting

### Issue: Back right wheel still not moving
**Check:**
```bash
# View raw log with turn commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# Terminal should show: TURN: ... RR=31 ...
```

**If RR shows 0 or wrong value:**
- Check Arduino wiring for M4 (rear right motor)
- Test Arduino directly: send `m 0:0:0:100` (should move only rear right)
- Check Arduino motor mapping (ensure M4 = rear right)

### Issue: Map still rotates with robot
**Solutions:**
1. **Check RViz Fixed Frame:**
   - Must be `map`, NOT `base_link`!
   - Top left dropdown in RViz

2. **Check TF tree:**
   ```bash
   ros2 run ros_arduino_bridge check_tf_tree.py
   # Verify map→odom→base_link chain exists
   ```

3. **Increase IMU covariances** (if EKF trusts IMU too much):
   ```yaml
   # In ekf_config.yaml, increase these:
   imu0_angular_velocity_covariance: [0.1, 0.0, 0.0, ...]  # Was 0.02
   ```

4. **Disable IMU angular velocity** (use orientation only):
   ```yaml
   # In ekf_config.yaml, imu0_config line:
   false, false, false,  # roll_vel, pitch_vel, yaw_vel (DISABLED)
   ```

### Issue: IMU calibration fails
**Symptoms:**
```
[WARN] IMU calibration failed - only X samples.
```

**Solutions:**
- Ensure robot is completely stationary during calibration
- Check Arduino serial connection (57600 baud)
- Test IMU manually: `ros2 topic echo /imu/data`
- Check Arduino 'i' command response (should return 9 values)

### Issue: Robot drifts during turns
**This is expected with wheel odometry!** The EKF should correct it.

**Verify EKF is working:**
```bash
# Compare raw odom vs fused odom
ros2 topic echo /odom --once
ros2 topic echo /odometry/filtered --once
# Fused should be more stable
```

---

## Expected Behavior After Fixes

### ✅ Motor Control
- All 4 wheels respond to turn commands
- In-place rotation works smoothly
- Terminal shows all PWM values during turns

### ✅ IMU
- Calibrates on startup (removes gyro bias)
- Publishes on `/imu/data` with `frame_id: imu_link`
- Covariances tuned for MPU6050 DMP

### ✅ TF Tree
- Complete chain: `map → odom → base_link → imu_link`
- Static transform published for IMU offset
- No TF warnings/errors

### ✅ SLAM/Mapping
- Map stays stationary when robot rotates
- Laser scan points don't move with robot base
- Clean map building without smearing

---

## Next Steps (Optional Tuning)

### 1. Measure Actual IMU Position
Update static transform in `sensor_fusion.launch.py`:
```python
# Current: IMU at (0, 0, 0.05) - 5cm above base_link center
# Measure your IMU position and update x, y, z values:
arguments=['x', 'y', 'z', '0', '0', '0', 'base_link', 'imu_link']
```

### 2. Tune EKF Covariances
Based on real-world testing, adjust in `ekf_config.yaml`:
- If map drifts: **increase** IMU covariances
- If odometry jumps: **decrease** wheel odometry covariances
- If turns are jerky: **increase** process noise

### 3. Record Test Data
```bash
# Record a test run for analysis
ros2 bag record -o test_run /odom /imu/data /scan /tf /odometry/filtered

# Play back later for tuning
ros2 bag play test_run
```

---

## Quick Reference Commands

```bash
# Build
cd ~/ros2_ws && colcon build --packages-select ros_arduino_bridge

# Run full stack
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false &
ros2 launch ros_arduino_bridge sensor_fusion.launch.py &
ros2 launch slam_toolbox online_async_launch.py &
rviz2

# Test turn
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Check TF
ros2 run ros_arduino_bridge check_tf_tree.py

# Monitor IMU
ros2 topic echo /imu/data
```

---

## Files Modified

1. **ros_arduino_bridge.py**
   - Added IMU calibration
   - Fixed cmd_vel_callback (4-wheel control)
   - Updated IMU covariances and frame_id
   - Normalized theta and quaternions

2. **sensor_fusion.launch.py**
   - Added static transform (base_link → imu_link)
   - Added launch arguments

3. **ekf_config.yaml**
   - Added `imu0_frame_id: imu_link`

4. **check_tf_tree.py** (NEW)
   - Diagnostic script for TF validation

---

## Success Criteria

- ✅ IMU calibration completes on startup
- ✅ All 4 wheels move during turns
- ✅ TF tree shows `imu_link` frame
- ✅ Map stays stationary in RViz when robot rotates
- ✅ No TF warnings/errors in terminal
- ✅ EKF publishes `/odometry/filtered`
- ✅ SLAM builds clean maps without smearing
