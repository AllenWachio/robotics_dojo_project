# Yaw Differentiation for Drift-Free Angular Velocity

## The Problem You Identified

When the robot stops rotating, the **map continues to rotate** even though the robot is stationary. This is caused by:

```
Raw Gyro → Reports -0.13 rad/s when stationary (drift!)
         ↓
       EKF integrates this "ghost rotation"
         ↓
    Position and map rotate continuously
```

## Why Raw Gyro Drifts

### Gyroscope Physics
- **Measures angular rate** by detecting Coriolis force
- **Subject to bias** - offset that accumulates over time
- **Temperature sensitive** - drift changes with temp
- **Integration amplifies error** - small drift → large rotation error

### Example Drift Impact
```
Gyro drift: 0.13 rad/s = 7.4°/s
Over 1 minute: 7.4° × 60 = 444° of rotation!
Robot appears to spin 1.2 full circles while sitting still!
```

---

## Your Solution: Differentiate DMP Yaw ✅

### Why This Works

1. **DMP Yaw is Drift-Free**
   - MPU6050's DMP fuses gyro + accelerometer
   - Accelerometer provides gravity reference (long-term stable)
   - Complementary filter eliminates gyro drift
   - Result: Stable orientation even after hours

2. **Differentiation Recovers Angular Velocity**
   ```
   angular_velocity = (yaw_now - yaw_previous) / time_difference
   ```

3. **No Drift Propagation**
   - Each angular velocity measurement is independent
   - No accumulated error from previous readings
   - Map stays stationary when robot stops!

---

## Mathematical Comparison

### Method 1: Raw Gyro (Old - Drifts!)

```python
# Read raw gyro from MPU6050 registers
gyro_z_raw = read_gyro_z()

# Convert to rad/s
gyro_z = (gyro_z_raw / 131.0) * (π/180)  # Has drift!

# Publish to EKF
imu_msg.angular_velocity.z = gyro_z
```

**Problem:** `gyro_z` includes drift (e.g., -0.13 rad/s when stationary)

---

### Method 2: Differentiate DMP Yaw (New - Drift-Free! ✅)

```python
# Read drift-free yaw from DMP
yaw_now = read_dmp_yaw()  # Radians, drift-free!

# Compute angular velocity by differentiation
dt = current_time - last_time
yaw_diff = yaw_now - last_yaw

# Handle angle wraparound (±π)
while yaw_diff > π:
    yaw_diff -= 2π
while yaw_diff < -π:
    yaw_diff += 2π

# Compute rate
gyro_z = yaw_diff / dt  # Drift-free angular velocity!

# Update for next iteration
last_yaw = yaw_now
last_time = current_time
```

**Advantage:** `gyro_z = 0.0` when robot is truly stationary!

---

## Implementation Details

### 1. State Variables Added

```python
# In __init__ (line ~112):
self.last_imu_yaw = None      # Previous yaw (radians)
self.last_imu_time = None     # Previous timestamp
```

### 2. Differentiation Logic

```python
# In _handle_imu_line (line ~265):

# Convert DMP yaw to radians
yaw = math.radians(yaw_deg)
current_time = self.get_clock().now()

if self.last_imu_yaw is not None:
    # Calculate time difference
    dt = (current_time - self.last_imu_time).nanoseconds / 1e9
    
    if dt > 0.001:  # Avoid division by zero
        # Calculate yaw change
        yaw_diff = yaw - self.last_imu_yaw
        
        # Normalize angle difference to [-π, π]
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Compute angular velocity
        gyro_z = yaw_diff / dt
        
        # Dead zone (reduce noise)
        if abs(gyro_z) < 0.01:  # ~0.57°/s
            gyro_z = 0.0
    else:
        gyro_z = 0.0
else:
    gyro_z = 0.0  # First reading

# Update for next iteration
self.last_imu_yaw = yaw
self.last_imu_time = current_time
```

### 3. Angle Wraparound Handling

**Critical for correctness!**

Without wraparound handling:
```
yaw transitions from +179° to -179° (crossing ±π boundary)
yaw_diff = -179° - (+179°) = -358°
gyro_z = -358°/dt → WRONG! (appears as fast reverse rotation)
```

With wraparound handling:
```
yaw_diff = -358°
while yaw_diff < -180°:
    yaw_diff += 360°  # Now: yaw_diff = 2°
gyro_z = 2°/dt → CORRECT! (small forward rotation)
```

---

## Why This is Better Than Using Accelerometer

You might think: "Can't we use accelerometer to detect rotation?"

**No, because:**

1. **Accelerometer measures linear acceleration**, not rotation
2. **During rotation at constant angular velocity:**
   - Centripetal acceleration appears: `a = ω² × r`
   - But this is ambiguous (could be linear motion on curve)
   - Can't distinguish rotation from curved path

3. **During angular acceleration:**
   - Tangential acceleration appears: `a = α × r`
   - But requires knowing `r` (distance from rotation center)
   - Noisy and unreliable for small robots

**DMP yaw differentiation is the correct approach!** ✅

---

## Expected Behavior After Fix

### Before (With Raw Gyro Drift)
```bash
$ ros2 topic echo /odometry/filtered

# Robot stationary, but:
twist:
  angular:
    z: -0.12945976331572662  # Ghost rotation!

# Position drifts:
pose:
  position:
    x: 0.0015  # Was 0.0
    y: 1.4269  # Was 0.0 - drifted 1.4m!
  orientation:
    z: -0.7683  # Constantly changing
```

### After (With Yaw Differentiation)
```bash
$ ros2 topic echo /odometry/filtered

# Robot stationary:
twist:
  angular:
    z: 0.0  # No ghost rotation! ✓

# Position stable:
pose:
  position:
    x: 0.0001  # Stays near zero
    y: 0.0002  # Stays near zero
  orientation:
    z: -0.7641  # Constant (not changing)
```

---

## Testing the Fix

### Test 1: Stationary Drift Test

```bash
# Terminal 1: Launch system
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Terminal 2: Monitor angular velocity
ros2 topic echo /imu/data | grep -A1 "angular_velocity:"

# Expected when robot is stationary:
angular_velocity:
  z: 0.0  # Should be zero or very close (< 0.01)
```

### Test 2: Rotation Then Stop Test

```bash
# Rotate robot slowly, then stop completely

# Monitor /odometry/filtered
ros2 topic echo /odometry/filtered

# Expected after stopping:
# - angular.z drops to 0.0 immediately
# - position.x and position.y stop changing
# - orientation.z becomes constant
```

### Test 3: Map Stability in RViz

```bash
# Start SLAM
ros2 launch slam_toolbox online_async_launch.py
rviz2

# In RViz:
# 1. Set Fixed Frame: map
# 2. Add LaserScan display
# 3. Rotate robot, then stop
# 4. Scan points should STOP moving when robot stops
```

---

## Comparison: Different Angular Velocity Sources

| Source | Drift | Accuracy (Stationary) | Accuracy (Moving) | Latency |
|--------|-------|---------------------|------------------|---------|
| **Raw Gyro** | ❌ High (~0.1 rad/s) | ❌ Poor | ✅ Good | ✅ Low |
| **Calibrated Gyro** | ⚠️ Medium (~0.01 rad/s) | ⚠️ Fair | ✅ Good | ✅ Low |
| **DMP Yaw Diff** ⭐ | ✅ None | ✅ Excellent | ✅ Good | ✅ Low |
| **Wheel Odometry** | ✅ None | ✅ Excellent | ❌ Poor (slip) | ⚠️ Medium |

**Winner:** DMP Yaw Differentiation! ✅

---

## Why Not Use Wheel Odometry for Angular Velocity?

You might ask: "Why not compute angular velocity from wheel encoders?"

```python
# From differential drive kinematics:
angular_velocity = (right_speed - left_speed) / base_width
```

**Problems:**

1. **Wheel slip during turns** - wheels skid, measurement wrong
2. **Encoder quantization** - coarse resolution at low speeds
3. **Delayed updates** - encoders polled at ~10Hz, IMU at 100Hz
4. **Stationary drift** - encoder noise accumulates

**DMP yaw differentiation avoids all these issues!**

---

## Technical Deep Dive: DMP Complementary Filter

### What the DMP Does Internally

```
     ┌──────────────────────────────┐
     │  MPU6050 DMP (on-chip)      │
     │                              │
Raw  │  ┌──────────┐                │
Gyro │  │ Integrate├──┬──→ Complementary  │
─────┼─→│  Gyro   │  │    Filter         │
     │  └──────────┘  │    ↓              │  Drift-Free
     │                │  ┌─────┐          │  Orientation
Raw  │  ┌──────────┐  └─→│ α·G │          │  (Yaw, Pitch, Roll)
Accel│  │ Gravity  │     │  +  ├─────────┼─→
─────┼─→│ Vector   ├────→│(1-α)│          │
     │  └──────────┘     │·A   │          │
     │                   └─────┘          │
     └──────────────────────────────────┘
```

**Complementary Filter Math:**
```
orientation = α × (gyro_integrated) + (1-α) × (accel_gravity)
```

Where:
- `α ≈ 0.98` - Trust gyro for short-term accuracy
- `(1-α) ≈ 0.02` - Trust accel for long-term stability

**Result:** Gyro provides fast response, accel prevents drift!

---

## Summary

### What Changed

**Before:**
```python
gyro_z = raw_gyro_z / scale  # Includes drift
```

**After:**
```python
gyro_z = (yaw_now - yaw_prev) / dt  # Drift-free!
```

### Why It Works

1. DMP yaw is **drift-free** (fused gyro + accel)
2. Differentiation **recovers angular velocity** accurately
3. Dead zone **eliminates noise** when stationary
4. Wraparound handling **prevents discontinuities**

### Expected Results

- ✅ Zero angular velocity when robot is stopped
- ✅ Map stays stationary in RViz
- ✅ No position drift when robot isn't moving
- ✅ Accurate angular velocity during actual rotation
- ✅ Clean SLAM maps without smearing

---

## Alternative: If DMP is Not Available

If your IMU doesn't have DMP, you can still use this approach by:

1. **Fusing gyro + accel yourself** (complementary filter)
2. **Using magnetometer** for absolute heading (if available)
3. **Computing yaw from wheel odometry** (less accurate)

**But since you have MPU6050 with DMP, use it!** It's the best solution.

---

## References

- MPU6050 DMP: [InvenSense Motion Driver](https://invensense.tdk.com/)
- Complementary Filter: [Mahony et al., 2008]
- Angle Differentiation: Standard numerical differentiation
- ROS robot_localization: Uses similar technique internally

---

## Testing Checklist

After deploying this fix, verify:

- [ ] `ros2 topic echo /imu/data` shows `angular_velocity.z ≈ 0.0` when stationary
- [ ] Robot doesn't "walk" during in-place rotation
- [ ] `/odometry/filtered` position stays constant when robot stops
- [ ] Map in RViz stops moving immediately when robot stops
- [ ] SLAM maps are clean without rotation smearing
- [ ] Covariance on yaw in `/odometry/filtered` stays reasonable (<100)

---

**Your intuition was spot-on!** Differentiating drift-free DMP yaw is the correct solution. 🎯
