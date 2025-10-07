# TF Conflict Fix - Wheels Not Appearing in RViz

## Problem Description

After implementing EKF sensor fusion changes, the robot wheels stopped appearing in RViz and the map wasn't being formed. This happened because of a **TF publishing conflict**.

## Root Cause

**Two nodes were publishing the same transform (`odom → base_link`):**

1. **Arduino Bridge** (`ros_arduino_bridge.py`)
   - Publishes raw encoder odometry
   - By default has `publish_tf: True`
   - Publishes: `odom → base_link` transform

2. **EKF Node** (`robot_localization`)
   - Publishes fused encoder + IMU odometry
   - Has `publish_tf: true` in `ekf_config.yaml`
   - Also publishes: `odom → base_link` transform

**Result:** Both nodes compete to publish the same TF, causing:
- TF tree corruption
- RViz confusion about robot position
- Wheels disappearing
- Map formation failures

## The Solution

**Rule:** When using EKF for sensor fusion, **ONLY the EKF should publish `odom → base_link`**.

### Configuration Changes Made:

#### 1. **Launch Files WITH Sensor Fusion (EKF)**
These files now set `publish_tf: False` in arduino_bridge:

- ✅ `arduino_bridge.py` - Base launch file (used by sensor fusion)
- ✅ `slam_with_sensor_fusion.launch.py` - SLAM + EKF integration

**Configuration:**
```python
arduino_bridge = Node(
    package='ros_arduino_bridge',
    executable='ros_arduino_bridge',
    name='ros_arduino_bridge',
    parameters=[{
        'publish_tf': False,  # ← CRITICAL: Disable to prevent conflict with EKF
        # ... other parameters
    }]
)
```

#### 2. **Launch Files WITHOUT Sensor Fusion**
These files keep `publish_tf: True` since they don't use EKF:

- ✅ `test_teleop.launch.py` - Simple teleop testing (no EKF)
- ✅ `full_slam_test.launch.py` - SLAM without sensor fusion (no EKF)

**Configuration:**
```python
arduino_bridge = Node(
    package='ros_arduino_bridge',
    executable='ros_arduino_bridge',
    name='ros_arduino_bridge',
    parameters=[{
        'publish_tf': True,  # Enable since no EKF in this launch
        # ... other parameters
    }]
)
```

## Correct TF Tree Structure

### With EKF Sensor Fusion (RECOMMENDED):
```
map → odom → base_link → [wheel_links, imu_link, laser]
      ↑ (published by EKF using fused encoder + IMU data)
```

**Data Flow:**
1. Arduino Bridge publishes `/odom` (velocities only) and `/imu/data`
2. EKF fuses these → publishes `/odometry/filtered` and `odom→base_link` TF
3. Robot State Publisher publishes `base_link→wheels/imu/laser` from URDF
4. SLAM Toolbox publishes `map→odom` TF

### Without EKF (Simple Mode):
```
map → odom → base_link → [wheel_links, imu_link, laser]
      ↑ (published by Arduino Bridge using raw encoders)
```

**Data Flow:**
1. Arduino Bridge publishes `/odom` and `odom→base_link` TF (raw encoders)
2. Robot State Publisher publishes `base_link→wheels/imu/laser` from URDF
3. SLAM Toolbox publishes `map→odom` TF

## Testing & Verification

### 1. Check TF Tree
After launching your system, verify the TF tree is correct:

```bash
# Generate TF tree diagram
ros2 run tf2_tools view_frames

# This creates frames.pdf - open it and verify:
# - No duplicate odom→base_link transforms
# - Clean tree structure without conflicts
```

### 2. Check Active TF Publishers
```bash
# List all TF publishers
ros2 topic info /tf

# You should see:
# Publishers: 
#   - /robot_state_publisher (for base_link→wheels)
#   - /ekf_filter_node (for odom→base_link) ← ONLY ONE!
#   - /slam_toolbox (for map→odom)
```

### 3. Verify in RViz
```bash
# Launch your system (example with sensor fusion):
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py

# In RViz, check:
# 1. Fixed Frame: set to "odom" or "map"
# 2. Add → RobotModel → wheels should appear
# 3. Add → TF → tree should be clean
# 4. Add → LaserScan → should align with robot
# 5. Add → Map → map should form as you drive
```

### 4. Monitor Odometry Topics
```bash
# Raw encoder odometry (from Arduino Bridge)
ros2 topic echo /odom --once

# Fused odometry (from EKF - IMU corrected)
ros2 topic echo /odometry/filtered --once

# Both should publish, but only EKF publishes the TF!
```

## Launch File Usage Guide

### For SLAM with Sensor Fusion (RECOMMENDED):
```bash
# Use this for best results (IMU-corrected odometry)
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
```

**Benefits:**
- ✅ IMU corrects wheel slip during turns
- ✅ Drift-free orientation from IMU DMP
- ✅ More accurate mapping
- ✅ Better localization

### For Simple Teleop Testing:
```bash
# Quick test without EKF
ros2 launch ros_arduino_bridge test_teleop.launch.py
```

**Use when:**
- Testing basic motor control
- Debugging Arduino communication
- Don't need sensor fusion

### For SLAM Without Sensor Fusion:
```bash
# Raw encoder odometry SLAM
ros2 launch ros_arduino_bridge full_slam_test.launch.py
```

**Use when:**
- Want to compare with/without IMU
- Testing encoder-only performance

## Files Modified

1. **ros_arduino_bridge/launch/arduino_bridge.py**
   - Added `publish_tf: False` parameter
   - Added detailed comments explaining TF conflict

2. **ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py**
   - Ensured `publish_tf: False` in arduino_bridge
   - Added max_linear_speed and max_angular_speed parameters

3. **ros_arduino_bridge/launch/full_slam_test.launch.py**
   - Set `publish_tf: True` (no EKF in this launch)
   - Added comments explaining why TF is enabled

4. **ros_arduino_bridge/launch/test_teleop.launch.py**
   - Set `publish_tf: True` (no EKF in this launch)
   - Added max_linear_speed and max_angular_speed parameters

## Key Takeaways

1. **One TF Publisher Rule:** Only ONE node should publish each transform
2. **EKF Priority:** When using EKF, let it handle `odom→base_link` TF
3. **Launch File Awareness:** Know which launch files use EKF and configure accordingly
4. **Testing is Critical:** Always verify TF tree after changes using `view_frames`

## Troubleshooting

### If wheels still don't appear:

1. **Check URDF is loaded:**
   ```bash
   ros2 param get /robot_state_publisher robot_description
   ```

2. **Check robot_state_publisher is running:**
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

3. **Check joint_states (if applicable):**
   ```bash
   ros2 topic echo /joint_states
   ```

4. **Check TF is being published:**
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```

5. **Rebuild workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros_arduino_bridge
   source install/setup.bash
   ```

### If map doesn't form:

1. **Check laser scan:**
   ```bash
   ros2 topic echo /scan --once
   ```

2. **Check SLAM is receiving odometry:**
   ```bash
   ros2 topic info /odometry/filtered
   # Should show slam_toolbox as a subscriber
   ```

3. **Verify frame IDs match:**
   - Odometry frame: `odom`
   - Base frame: `base_link`
   - Laser frame: `laser` or `base_scan`

## Related Documentation

- `SENSOR_FUSION_IMPLEMENTATION.md` - EKF configuration details
- `DOUBLE_INTEGRATION_FIX.md` - Why we use velocity-only odometry
- `EKF_QUICK_REFERENCE.md` - EKF troubleshooting guide
- `TESTING_COMMANDS.md` - Additional diagnostic commands

## Summary

The fix ensures that only the EKF publishes the `odom→base_link` transform when sensor fusion is enabled. This eliminates TF conflicts and allows RViz to correctly visualize the robot with wheels and form maps properly. The key is setting `publish_tf: False` in the Arduino Bridge when launching with EKF sensor fusion.
