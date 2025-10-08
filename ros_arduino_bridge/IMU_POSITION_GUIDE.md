# IMU Position Configuration Guide

## Why IMU Position Matters

The EKF (Extended Kalman Filter) needs to know **exactly where the IMU is mounted** relative to the robot's `base_link` frame. This is critical because:

1. **Accurate sensor fusion** - EKF transforms IMU measurements to the robot's center
2. **Correct angular velocity** - If IMU is offset, rotations create linear acceleration artifacts
3. **Proper gravity removal** - Gravity vector depends on IMU orientation and position

## Understanding the Coordinate System

From the **robot's perspective** (standing behind it, looking forward):

```
        +x (forward)
         ↑
         |
         |
+y ← --- o --- (robot center = base_link)
(left)   |
         |
         ↓
        -x (backward)

+z = UP (above base_link)
-z = DOWN (below base_link)
```

## Current Configuration

**File:** `launch/sensor_fusion.launch.py`

```python
arguments=['0', '0', '-0.03', '0', '0', '0', 'base_link', 'imu_link']
#          x    y    z        roll pitch yaw  parent    child
```

**Current assumption:** IMU is **3cm below** base_link center (z = -0.03)

---

## How to Measure Your IMU Position

### Method 1: Direct Measurement (Most Accurate)

1. **Define base_link position:**
   - Usually the geometric center of the robot chassis
   - Or the center point between the wheels
   - Mark this point with tape/marker

2. **Measure from base_link to IMU:**
   ```
   Measure in cm, then convert to meters:
   
   x (forward/back):  _____ cm = _____ m
   y (left/right):    _____ cm = _____ m  
   z (up/down):       _____ cm = _____ m
   ```

3. **Apply signs:**
   - If IMU is **forward** of base_link → x is **positive**
   - If IMU is **backward** → x is **negative**
   - If IMU is **left** → y is **positive**
   - If IMU is **right** → y is **negative**
   - If IMU is **above** → z is **positive**
   - If IMU is **below** → z is **negative** ← **YOUR CASE!**

### Method 2: Using a Level Surface

1. Place robot on flat surface
2. Measure from ground to base_link: _____ cm
3. Measure from ground to IMU: _____ cm
4. Difference = z offset
5. If IMU is lower than base_link → z is negative

---

## Example Configurations

### Example 1: IMU Mounted Below Chassis
```
IMU is 3cm below base_link, centered horizontally
```
```python
arguments=['0', '0', '-0.03', '0', '0', '0', 'base_link', 'imu_link']
```

### Example 2: IMU Below and Forward
```
IMU is 5cm below, 2cm forward of center
```
```python
arguments=['0.02', '0', '-0.05', '0', '0', '0', 'base_link', 'imu_link']
```

### Example 3: IMU Below, Forward, and Left
```
IMU is 4cm below, 3cm forward, 1cm to left
```
```python
arguments=['0.03', '0.01', '-0.04', '0', '0', '0', 'base_link', 'imu_link']
```

---

## How to Update the Configuration

### Step 1: Measure Your IMU Position

Fill in your measurements:
```
x = _____ m  (forward = +, backward = -)
y = _____ m  (left = +, right = -)
z = _____ m  (up = +, down = -)
```

### Step 2: Edit Launch File

**File:** `ros_arduino_bridge/launch/sensor_fusion.launch.py`

Find line ~58 and update:
```python
arguments=['x_value', 'y_value', 'z_value', '0', '0', '0', 'base_link', 'imu_link']
```

### Step 3: Rebuild and Test

```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash

# Test the new transform
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# In another terminal, verify transform
ros2 run tf2_ros tf2_echo base_link imu_link

# Should show:
# Translation: [x, y, z]  ← Your values!
```

---

## Impact of Incorrect IMU Position

### ❌ If Position is Wrong:

1. **During straight motion:**
   - EKF will see false rotation (from linear accel at offset IMU)
   - Odometry will curve slightly

2. **During rotation:**
   - EKF will see false linear acceleration
   - Position estimate will drift outward from true path

3. **During acceleration:**
   - Gravity removal will be incorrect
   - Vertical acceleration artifacts appear as horizontal motion

### ✅ If Position is Correct:

1. Smooth, accurate odometry during all motions
2. No drift during in-place rotations
3. Clean SLAM maps without smearing

---

## Testing IMU Position Accuracy

### Test 1: In-Place Rotation
```bash
# Start everything
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Monitor fused odometry
ros2 topic echo /odometry/filtered

# Rotate robot 360° in place (teleop or manually)
# Check final position - should be VERY close to starting position
# If robot "walks" during rotation, IMU position may be wrong
```

### Test 2: Straight Line
```bash
# Drive robot straight for 2 meters
# Path in /odometry/filtered should be perfectly straight
# If path curves, IMU position likely has x or y offset error
```

### Test 3: Check Acceleration
```bash
# Watch IMU while robot is stationary
ros2 topic echo /imu/data

# linear_acceleration.z should be ~9.81 m/s² (gravity)
# If significantly different, IMU orientation or position is wrong
```

---

## Advanced: IMU Orientation

**Note:** The current setup assumes IMU is mounted **flat** (z-axis pointing up).

If your IMU is rotated, you need to adjust the roll/pitch/yaw values:

```python
# Example: IMU rotated 90° around x-axis (upside down)
arguments=['0', '0', '-0.03', '3.14159', '0', '0', 'base_link', 'imu_link']
#                              roll=180°
```

**Most common case:** IMU is flat → roll=0, pitch=0, yaw=0 ✓

---

## Quick Reference

| IMU Location | x | y | z |
|-------------|---|---|---|
| 3cm below center | 0 | 0 | **-0.03** |
| 5cm below, 2cm forward | 0.02 | 0 | -0.05 |
| 4cm below, 1cm left | 0 | 0.01 | -0.04 |
| 2cm above center | 0 | 0 | 0.02 |

## Your Robot's Configuration

**TODO: Fill this in after measuring!**

```
IMU Position:
  x = _____ m  (measured from base_link, forward is positive)
  y = _____ m  (measured from base_link, left is positive)
  z = _____ m  (measured from base_link, UP is positive, DOWN is negative)

Base Link Definition:
  [ ] Center of robot chassis
  [ ] Center between wheels
  [ ] Other: _________________

IMU Orientation:
  [ ] Flat (z-axis up) - most common
  [ ] Rotated: roll=___ pitch=___ yaw=___
```

**Updated launch file command:**
```python
arguments=['____', '____', '____', '0', '0', '0', 'base_link', 'imu_link']
```

---

## Verification Checklist

After updating IMU position:

- [ ] Rebuilt workspace: `colcon build --packages-select ros_arduino_bridge`
- [ ] Verified transform: `ros2 run tf2_ros tf2_echo base_link imu_link`
- [ ] Transform shows correct x, y, z values
- [ ] Ran in-place rotation test - robot stays in same spot
- [ ] Ran straight line test - path is straight in RViz
- [ ] IMU z-acceleration reads ~9.81 m/s² when stationary
- [ ] SLAM map is clean without smearing during turns

---

## Need Help?

If you're unsure about your measurements:

1. **Take a photo** of your robot from the side showing IMU and base_link
2. **Measure dimensions** with a ruler/caliper
3. **Start conservative:** Use small offset like z=-0.01 and test
4. **Iterate:** Adjust based on test results (rotation drift, path curvature)

**Remember:** A slightly wrong offset is better than no offset!
The EKF is robust and will still improve odometry even if position is off by 1-2cm.
