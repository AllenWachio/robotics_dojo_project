# Map Breaking During Turns - Diagnostic & Solutions

## Problem Summary

The map breaks/distorts when the robot makes turns despite having IMU sensor fusion implemented. This is a common issue in SLAM systems and can have multiple root causes.

## Current Implementation Status ✅

Your setup includes:

- ✅ Full 9-DOF IMU implementation (yaw, pitch, roll, gyro, accel)
- ✅ EKF sensor fusion configured (`ekf_config.yaml`)
- ✅ Odometry covariances properly set
- ✅ SLAM Toolbox configured
- ✅ Proper frame IDs and TF tree

## Root Cause Analysis

### Most Likely Causes (in order of probability):

### 🔴 **1. EKF Not Running or SLAM Not Using Filtered Odometry**

**Symptoms:**

- Map breaks during turns
- `/odometry/filtered` topic may not exist
- SLAM uses raw `/odom` instead of filtered

**Diagnosis:**

```bash
# Check if EKF is running
ros2 node list | grep ekf

# Check if filtered odometry exists
ros2 topic list | grep odometry

# Check what SLAM is subscribed to
ros2 topic info /odom
ros2 topic info /odometry/filtered
```

**Solution:**
You need to launch BOTH the arduino bridge AND the EKF sensor fusion, then configure SLAM to use the filtered odometry.

**Fix Implementation:**

1. Create an integrated launch file
2. Configure SLAM to use `/odometry/filtered`

---

### 🟡 **2. TF Publishing Conflict (Critical!)**

**Problem:**
Both `ros_arduino_bridge` and `ekf_node` may be publishing the `odom→base_link` transform simultaneously, causing conflicts.

**Symptoms:**

- Jumpy robot pose
- TF warnings about multiple publishers
- Map breaks randomly

**Diagnosis:**

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor TF warnings
ros2 launch ros_arduino_bridge arduino_bridge.py 2>&1 | grep -i "tf"
```

**Solution:**
When using EKF, disable TF publishing from arduino_bridge.

---

### 🟠 **3. IMU Data Quality Issues**

**Problem:**
IMU may be sending noisy/incorrect data or not being used by EKF.

**Symptoms:**

- IMU values don't change when robot rotates
- Large spikes in IMU data
- Orientation drifts even when stationary

**Diagnosis:**

```bash
# Check IMU data quality
ros2 topic echo /imu/data

# Rotate robot manually and watch orientation change
ros2 topic echo /imu/data/orientation

# Check IMU publishing rate
ros2 topic hz /imu/data  # Should be ~5Hz
```

**Look for:**

- Orientation quaternion should change smoothly when rotating
- Angular velocity should be non-zero during rotation
- Linear acceleration should be ~9.8 m/s² in Z when stationary

---

### 🟢 **4. SLAM Parameters Too Aggressive**

**Problem:**
SLAM Toolbox may be too sensitive or have incorrect parameters for your robot's dynamics.

**Symptoms:**

- Map breaks specifically during fast turns
- Works fine with slow movements
- Loop closures cause large corrections

**Common Parameter Issues:**

- `minimum_travel_distance: 0.05` - May be too small, causing too many scans
- `minimum_travel_heading: 0.15` - May cause issues during sharp turns
- `minimum_time_interval: 0.2` - Too short for slow odometry updates

---

## Complete Solution Implementation

### **Step 1: Create Integrated Launch File with EKF**

Create a new launch file that properly starts everything:

```bash
cd /home/allen-wachio/ros2_ws/src/ros_arduino_bridge/launch
```

**File: `slam_with_sensor_fusion.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    pkg_share = FindPackageShare('ros_arduino_bridge')

    # Configuration files
    slam_params_file = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])
    ekf_params_file = PathJoinSubstitution([pkg_share, 'config', 'ekf_config.yaml'])
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'new_robot_urdf.xacro'])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': False
        }]
    )

    # Arduino Bridge - DISABLE TF publishing (EKF will handle it)
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'baud_rate': 57600,
            'base_width': 0.249000,
            'wheel_radius': 0.042500,
            'encoder_ticks_per_rev': 373,
            'publish_tf': False  # ⚠️ CRITICAL: Let EKF publish TF
        }]
    )

    # EKF Sensor Fusion Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file]
    )

    # SLAM Toolbox - Use filtered odometry
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
        remappings=[
            ('/odom', '/odometry/filtered')  # ⚠️ CRITICAL: Use filtered odometry!
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        arduino_bridge,
        ekf_node,
        slam_toolbox_node
    ])
```

---

### **Step 2: Verify EKF Configuration**

Check that your `ekf_config.yaml` has these critical settings:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 20.0 # or 30.0
    two_d_mode: true
    publish_tf: true # ⚠️ EKF publishes odom→base_link

    # Use wheel odometry for position/velocity
    odom0: /odom
    odom0_config: [
        true,
        true,
        false, # x, y position (use encoders)
        false,
        false,
        false, # orientation (DON'T use - slips!)
        true,
        true,
        false, # x_vel, y_vel (use encoders)
        false,
        false,
        false, # angular velocities
        false,
        false,
        false, # accelerations
      ]

    # Use IMU for orientation and angular velocity
    imu0: /imu/data
    imu0_config: [
        false,
        false,
        false, # position
        true,
        true,
        true, # orientation (USE for heading!)
        false,
        false,
        false, # velocities
        true,
        true,
        true, # angular velocities (USE - corrects turns!)
        true,
        true,
        true, # accelerations
      ]
```

---

### **Step 3: Tune SLAM Parameters for Turns**

Update `mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Make SLAM more tolerant during turns
    minimum_travel_distance: 0.10 # Increase from 0.05
    minimum_travel_heading: 0.25 # Increase from 0.15 (~14 degrees)
    minimum_time_interval: 0.3 # Increase from 0.2

    # Trust odometry more (since it's now fused with IMU)
    distance_variance_penalty: 0.05 # Lower = trust more
    angle_variance_penalty: 0.15 # Lower = trust more

    # Scan matching parameters
    correlation_search_space_dimension: 0.3 # Reduce search space
    coarse_search_angle_offset: 0.15
    fine_search_angle_offset: 0.001

    # Make loop closure less aggressive
    loop_match_minimum_response_fine: 0.55 # Increase from 0.45
```

---

### **Step 4: Add IMU Link to URDF (Optional but Recommended)**

Add this to your `new_robot_urdf.xacro`:

```xml
<!-- IMU Link -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
</joint>
```

---

## Testing Protocol

### **Phase 1: Verify Sensor Fusion**

```bash
# Terminal 1: Launch complete system
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py

# Terminal 2: Verify all topics
ros2 topic list | grep -E "odom|imu"
# Expected:
#   /odom                 (raw encoders)
#   /imu/data             (IMU)
#   /odometry/filtered    (fused output)

# Terminal 3: Check publishing rates
ros2 topic hz /odom              # ~10Hz
ros2 topic hz /imu/data          # ~5Hz
ros2 topic hz /odometry/filtered # ~20Hz

# Terminal 4: Verify no TF conflicts
ros2 run tf2_ros tf2_monitor
# Should show clean tree: map → odom → base_link → imu_link
```

---

### **Phase 2: Compare Raw vs Filtered Odometry**

```bash
# Rotate robot 360° in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once

# Terminal 1: Raw odometry
ros2 topic echo /odom/pose/pose/orientation

# Terminal 2: Filtered odometry
ros2 topic echo /odometry/filtered/pose/pose/orientation

# Expected: Filtered should be smoother and more accurate
```

---

### **Phase 3: Real Mapping Test**

```bash
# Start system
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py

# In another terminal: Open RViz
rviz2

# Configure RViz:
# 1. Set Fixed Frame: "map"
# 2. Add → Map → Topic: /map
# 3. Add → LaserScan → Topic: /scan
# 4. Add → TF
# 5. Add → Odometry → Topic: /odometry/filtered

# Drive robot:
# - Make slow 90° turn
# - Make fast 180° spin
# - Drive in figure-8 pattern

# Observe:
# ✅ Map should stay consistent
# ✅ No distortion during turns
# ✅ Scan alignment remains good
```

---

### **Phase 4: Diagnostic Checks**

#### **Check 1: IMU Contributing to Fusion**

```bash
# Compare during 90° rotation
ros2 topic echo /odom/pose/pose/orientation/z  # Will drift
ros2 topic echo /odometry/filtered/pose/pose/orientation/z  # Should be accurate
```

#### **Check 2: No TF Warnings**

```bash
# Should see NO warnings about multiple TF publishers
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py 2>&1 | grep -i "warn"
```

#### **Check 3: EKF Status**

```bash
# EKF should show healthy fusion
ros2 topic echo /diagnostics --once
# Look for ekf_filter_node diagnostics
```

---

## Troubleshooting Guide

### **Problem: Map still breaks during turns**

**Likely causes:**

1. EKF not running → Check `ros2 node list | grep ekf`
2. SLAM not using filtered odometry → Check remapping in launch file
3. TF conflict → Check `publish_tf: False` in arduino_bridge

**Debug:**

```bash
# Check what SLAM is actually subscribed to
ros2 topic info /odom --verbose
# Should show slam_toolbox subscribed to /odometry/filtered (not /odom)
```

---

### **Problem: /odometry/filtered not publishing**

**Causes:**

- robot_localization not installed
- EKF configuration error
- IMU or odometry topics missing

**Fix:**

```bash
# Install robot_localization
sudo apt update
sudo apt install ros-humble-robot-localization

# Check inputs
ros2 topic hz /odom
ros2 topic hz /imu/data

# Restart with debug logging
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py --log-level DEBUG
```

---

### **Problem: IMU data is all zeros**

**Causes:**

- Arduino not sending 9-value format
- Serial communication issue

**Fix:**

```bash
# Test Arduino directly
# Connect to serial monitor at 57600 baud
# Type: i
# Expected: 45.30 2.10 -1.50 0.05 -0.02 12.34 0.12 -0.05 9.78

# Check bridge logs
ros2 launch ros_arduino_bridge arduino_bridge.py --log-level DEBUG | grep IMU
```

---

### **Problem: Robot jumps/teleports during turns**

**Cause:** Multiple TF publishers (CRITICAL!)

**Fix:**

```bash
# Verify arduino_bridge is NOT publishing TF
ros2 param get /ros_arduino_bridge publish_tf
# Should return: False

# Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf - should show single path: map→odom→base_link
```

---

## Advanced Tuning

### **If turns are too slow/conservative:**

In `mapper_params_online_async.yaml`:

```yaml
minimum_travel_heading: 0.15 # Reduce (more scans during turns)
angle_variance_penalty: 0.1 # Lower (trust odometry more)
```

### **If turns cause drift:**

In `ekf_config.yaml`:

```yaml
# Trust IMU orientation more
imu0_orientation_covariance: [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.02] # Lower yaw covariance
```

### **If map is too detailed/noisy:**

In `mapper_params_online_async.yaml`:

```yaml
minimum_time_interval: 0.5 # Increase (fewer scans)
resolution: 0.07 # Increase (coarser map)
```

---

## Quick Fix Checklist

Use this before testing:

- [ ] `robot_localization` installed
- [ ] IMU sending 9-value data format
- [ ] `/imu/data` topic publishing at ~5Hz with valid data
- [ ] `/odom` topic has non-zero covariances
- [ ] Arduino bridge has `publish_tf: False`
- [ ] EKF config has `publish_tf: true`
- [ ] SLAM remapped to `/odometry/filtered`
- [ ] `/odometry/filtered` topic exists and publishes at ~20Hz
- [ ] TF tree shows single path (no conflicts)
- [ ] URDF includes `imu_link` or IMU uses `base_link`

---

## Expected Results After Fix

✅ **Smooth turns**: Robot rotates without map distortion  
✅ **Accurate heading**: Orientation stays consistent  
✅ **No jumps**: No sudden position corrections  
✅ **Loop closure**: Can revisit areas without breaking map  
✅ **Scan alignment**: Lidar scans match environment during motion

---

## Monitoring Commands (Keep Running)

```bash
# Terminal 1: Launch system
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py

# Terminal 2: Monitor odometry comparison
watch -n 0.5 'ros2 topic echo /odometry/filtered/pose/pose/orientation --once | head -6'

# Terminal 3: Monitor TF
ros2 run tf2_ros tf2_echo odom base_link

# Terminal 4: RViz visualization
rviz2
```

---

## Next Steps

1. **Create the integrated launch file** (`slam_with_sensor_fusion.launch.py`)
2. **Test sensor fusion** without SLAM first
3. **Verify filtered odometry** is smooth during turns
4. **Enable SLAM** with filtered odometry
5. **Map a room** with multiple turns to validate
6. **Tune parameters** based on results

---

## Additional Resources

- [Robot Localization Troubleshooting](http://docs.ros.org/en/humble/p/robot_localization/index.html)
- [SLAM Toolbox Parameter Guide](https://github.com/SteveMacenski/slam_toolbox)
- Your project docs:
  - `SENSOR_FUSION_SUMMARY.md`
  - `EKF_IMPLEMENTATION_REVIEW.md`
  - `IMU_IMPLEMENTATION.md`

---

**Summary:** The most likely issue is that SLAM is using raw `/odom` instead of `/odometry/filtered`, and/or there's a TF publishing conflict. The integrated launch file above solves both issues.
