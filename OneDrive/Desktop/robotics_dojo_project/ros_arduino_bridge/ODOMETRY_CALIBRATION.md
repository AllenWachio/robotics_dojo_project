# Odometry Calibration Results

**Date:** October 5, 2025  
**Robot:** ROS Arduino Bridge - 4WD Robot  
**Test:** 1 meter straight-line movement

---

## Test Data

### Encoder Readings

| Motor            | Initial | Final  | Change (ticks) |
| ---------------- | ------- | ------ | -------------- |
| M1 (Front Left)  | 33,392  | 35,070 | 1,678          |
| M2 (Front Right) | 20,765  | 22,447 | 1,682          |
| M3 (Rear Left)   | 29,564  | 31,235 | 1,671          |
| M4 (Rear Right)  | 23,274  | 24,940 | 1,666          |

**Average:** 1,674.25 ticks per meter

### Physical Parameters

- **Distance traveled:** 1.0 meter (measured)
- **Wheel diameter:** 85mm (0.085m)
- **Wheel radius:** 42.5mm (0.0425m)
- **Wheel circumference:** 2π × 0.0425 = 0.2670m
- **Base width:** 208mm (0.208m)

---

## Calculations

### Encoder Ticks Per Revolution

```
ticks_per_rev = (average_ticks_per_meter) × (wheel_circumference)
ticks_per_rev = 1674.25 × 0.2670
ticks_per_rev = 447 ticks/revolution
```

### Previous vs Corrected Values

| Parameter             | Previous  | Corrected | Error          |
| --------------------- | --------- | --------- | -------------- |
| encoder_ticks_per_rev | 870       | 447       | 94.6% too high |
| Distance per tick     | 0.000306m | 0.000597m | 2× difference  |

---

## Root Cause Analysis

### Why Odometry Was Wrong:

1. **Encoder ticks setting was 870** (likely a typo or incorrect motor spec)
2. **Actual encoder ticks are 447** (measured empirically)
3. **Result:** Robot moved ~2× faster than RViz calculated

### Impact on Robot Behavior:

- ✅ Physical robot appeared faster than RViz visualization
- ✅ Maps kept changing/drifting during movement
- ✅ Turns were drastically wrong (error compounded in rotation calculations)
- ✅ SLAM couldn't build consistent maps

---

## Solution Applied

### Files Updated:

1. **`launch/arduino_bridge.py`**

   - Changed `encoder_ticks_per_rev: 870` → `447`

2. **`config/robot_params.yaml`**
   - Changed `encoder_ticks_per_rev: 870` → `447`

### Verification Formula:

```python
# Distance calculation in code:
ticks_to_meters = (2 * π * wheel_radius) / encoder_ticks_per_rev
ticks_to_meters = (2 * π * 0.0425) / 447
ticks_to_meters = 0.000597 m/tick

# For 1674.25 ticks (average):
distance = 1674.25 × 0.000597 = 1.0 meter ✅
```

---

## Next Steps for Further Tuning

### 1. Verify Linear Movement Accuracy

```bash
# Drive robot 2 meters, check if RViz matches
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# Stop after exactly 2 meters
# Check /odom should show ~2.0m traveled
```

### 2. Calibrate Turning Accuracy (Base Width)

If turns are still slightly off:

```bash
# Command 360° rotation
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# Measure actual rotation angle
# If RViz shows 360° but robot turns 380°:
# Adjust base_width = 0.208 × (360/380) = 0.197m
```

### 3. Test SLAM Mapping

```bash
# Launch SLAM and drive around
ros2 launch ros_arduino_bridge slam_launch.py

# Map should now be stable and consistent
# No more drifting or changing during movement
```

### 4. Monitor Encoder Symmetry

Check if all wheels contribute equally:

```bash
ros2 topic echo /raw_encoders
# All 4 motors should show similar tick counts for straight movement
# Large differences indicate mechanical issues (slipping, friction, misalignment)
```

---

## Troubleshooting

### If linear movement still incorrect:

- **Robot moves less than RViz:** Increase `encoder_ticks_per_rev`
- **Robot moves more than RViz:** Decrease `encoder_ticks_per_rev`
- **Adjust in increments of 10-20 ticks**

### If turns are still wrong:

- **Robot turns more than expected:** Decrease `base_width`
- **Robot turns less than expected:** Increase `base_width`
- **Adjust in increments of 0.005m (5mm)**

### If one side consistently different:

- **Check wheel slippage** (try on different surface)
- **Check encoder connections** (loose wires)
- **Check motor power balance** (PWM calibration)
- **Check mechanical alignment** (wheels parallel)

---

## Success Criteria

After calibration, you should observe:

- ✅ RViz robot position matches physical robot
- ✅ Odometry drift < 5% over 10 meters
- ✅ 360° turn shows ~360° rotation (±5°)
- ✅ SLAM maps are stable and consistent
- ✅ Encoder counts are balanced across all 4 wheels

---

## Notes

- **Motor Model:** 25GA370 DC 12V with D-shaped Hall encoder
- **Encoder Type:** Hall effect (quadrature)
- **Original incorrect value:** 870 ticks/rev (source unknown)
- **Measured correct value:** 447 ticks/rev
- **Calibration method:** Empirical measurement (most accurate)

## References

- Test performed: 1 meter straight-line movement
- Encoder data: `/raw_encoders` topic
- Odometry topic: `/odom`
- Transform: `odom` → `base_link`
