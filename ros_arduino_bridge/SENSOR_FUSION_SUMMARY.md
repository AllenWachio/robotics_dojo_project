# Sensor Fusion Implementation Summary

## What Was Updated

This update implements **full 9-DOF IMU sensor fusion** with wheel odometry to eliminate wheel slippage errors during turns.

### Files Modified/Created

#### 1. **ros_arduino_bridge.py** (UPDATED)
- Changed IMU command from `z` → `i`
- Updated `_handle_imu_line()` to parse 9 values: `yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z`
- Full quaternion computation from roll/pitch/yaw (ZYX convention)
- Populated angular velocity (gyro) and linear acceleration fields
- Added covariance matrices for EKF integration
- Changed frame_id to `imu_link`

#### 2. **ekf_config.yaml** (NEW)
Complete robot_localization EKF configuration:
- **Input 1**: `/odom` (wheel encoders) - linear velocities
- **Input 2**: `/imu/data` (IMU) - orientation, angular velocity, accelerations
- **Output**: `/odometry/filtered` - fused estimate correcting wheel slip
- Configured in 2D mode for differential drive on flat ground
- Tuned covariances to trust IMU angular velocity over wheel rotation during turns

#### 3. **sensor_fusion.launch.py** (NEW)
Launch file to start the EKF node with proper configuration

#### 4. **IMU_IMPLEMENTATION.md** (UPDATED)
Complete documentation with:
- 9-value Arduino protocol specification
- Testing procedures
- Troubleshooting guide
- Sensor fusion setup instructions

## Arduino Requirements

Your Arduino firmware **must** respond to command `i` with 9 space-separated values:

```
yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z
```

**Units:**
- Angles (yaw, pitch, roll): degrees
- Gyro (gyro_x/y/z): degrees/second
- Accel (accel_x/y/z): m/s²

**Example response:**
```
45.30 2.10 -1.50 0.05 -0.02 12.34 0.12 -0.05 9.78
```

## How It Works

### Data Flow
```
Arduino IMU (MPU6050/MPU9250)
    ↓ (send 'i' command @ 5Hz)
    ↓ (9-value response)
ROS2 Bridge (ros_arduino_bridge.py)
    ↓ (/imu/data topic)
    ↓
EKF Node (robot_localization)  ← also receives /odom from wheel encoders
    ↓
/odometry/filtered (corrected pose/velocity)
```

### Why This Fixes Wheel Slippage

**Problem**: During turns, differential drive wheels slip/skid, causing wheel odometry to drift.

**Solution**: 
- **Wheel encoders** provide accurate **linear velocity** (straight motion)
- **IMU gyroscope** provides accurate **angular velocity** (rotation - no slip!)
- **EKF** fuses both, trusting IMU for rotation and wheels for translation
- Result: Accurate odometry even during aggressive turns

## Testing Instructions

### On Raspberry Pi

#### 1. Install robot_localization (if not installed)
```bash
sudo apt update
sudo apt install ros-humble-robot-localization
```

#### 2. Rebuild ros_arduino_bridge
```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

#### 3. Test IMU Data
```bash
# Terminal 1: Start bridge
ros2 launch ros_arduino_bridge arduino_bridge.py

# Terminal 2: Check IMU topic
ros2 topic echo /imu/data --once

# Expected: Full IMU message with orientation, angular_velocity, linear_acceleration
```

If you see the full message with non-zero values → **IMU is working!**

#### 4. Start Sensor Fusion
```bash
# Terminal 1: Bridge (already running)
ros2 launch ros_arduino_bridge arduino_bridge.py

# Terminal 2: EKF sensor fusion
ros2 launch ros_arduino_bridge sensor_fusion.launch.py
```

#### 5. Monitor Fused Odometry
```bash
# Check topics
ros2 topic list | grep odom
# Should see: /odom (raw) and /odometry/filtered (fused)

# Compare raw vs fused
ros2 topic echo /odometry/filtered

# Check publishing rate
ros2 topic hz /odometry/filtered  # Should be ~30Hz
```

#### 6. Test Turn Accuracy
```bash
# Drive robot in a circle and compare:
# Terminal 1: Raw wheel odometry
ros2 topic echo /odom/pose/pose/orientation

# Terminal 2: Fused odometry
ros2 topic echo /odometry/filtered/pose/pose/orientation

# The filtered output should be more stable and accurate during turns
```

## Verifying It's Working

### Quick Test
1. Start bridge + EKF
2. Drive robot forward 1 meter → both /odom and /odometry/filtered should match
3. Rotate robot 360° in place → /odometry/filtered should return to ~0° heading (corrected by IMU), /odom might drift

### Signs of Success
✅ `/imu/data` publishes at ~5Hz with full data  
✅ `/odometry/filtered` publishes at ~30Hz  
✅ During pure rotation, filtered heading tracks better than raw odometry  
✅ No TF errors or warnings in logs  
✅ EKF node shows in `ros2 node list`

### Signs of Problems
❌ IMU topic shows all zeros → Arduino not sending correct format  
❌ EKF warns about covariance → tune values in ekf_config.yaml  
❌ TF errors → check frame_id matches (`imu_link` in all places)  
❌ Filtered odometry identical to raw → EKF not fusing (check config)

## Tuning (Optional)

If the fused odometry behaves strangely:

### 1. Adjust IMU Covariances
In `ros_arduino_bridge.py`, `_handle_imu_line()`:
```python
# Lower = trust more, Higher = trust less
orientation_covariance = [0.01, ...]  # Tune based on IMU specs
angular_velocity_covariance = [0.001, ...]
linear_acceleration_covariance = [0.01, ...]
```

### 2. Adjust EKF Process Noise
In `ekf_config.yaml`:
```yaml
process_noise_covariance: [
  0.05,  # x position - increase if too jumpy, decrease if too sluggish
  ...
]
```

### 3. Change Sensor Trust
In `ekf_config.yaml`, modify which values are fused:
```yaml
odom0_config: [false, false, ..., true, ...]  # Enable/disable specific values
imu0_config: [false, false, ..., true, ...]
```

## Integration with SLAM

The fused odometry can be used as input to SLAM:

```bash
# Use filtered odometry for SLAM instead of raw
ros2 launch slam_toolbox online_async_launch.py \
  odom_topic:=/odometry/filtered
```

This provides SLAM with more accurate odometry, improving map quality.

## Next Steps

1. **Deploy**: Push changes to GitHub, pull on Pi, rebuild
2. **Test Arduino**: Verify `i` command returns 9 values
3. **Test IMU Topic**: Confirm `/imu/data` has full data
4. **Start EKF**: Launch sensor fusion and monitor filtered output
5. **Validate**: Drive robot and compare raw vs filtered odometry
6. **Tune**: Adjust covariances if needed

## Troubleshooting Guide

### Problem: IMU topic shows zeros
**Solution**: Arduino not sending 9-value format. Check serial monitor response to `i` command.

### Problem: EKF not starting
**Solution**: 
```bash
sudo apt install ros-humble-robot-localization
# Then rebuild workspace
```

### Problem: TF errors "imu_link not found"
**Solution**: Check URDF defines `imu_link` frame, or change frame_id to `base_link` everywhere.

### Problem: Filtered = Raw (no fusion happening)
**Solution**: Check EKF config, ensure `imu0_config` and `odom0_config` enable the right values.

### Problem: Unstable filtered output
**Solution**: Increase process_noise_covariance values in ekf_config.yaml.

## Files Summary

```
ros_arduino_bridge/
├── ros_arduino_bridge/
│   └── ros_arduino_bridge.py          # UPDATED: Full 9-DOF IMU parsing
├── config/
│   ├── ekf_config.yaml                # NEW: EKF sensor fusion config
│   └── robot_params.yaml              # (already has imu_frame)
├── launch/
│   └── sensor_fusion.launch.py        # NEW: Launch EKF node
├── IMU_IMPLEMENTATION.md              # UPDATED: Full documentation
└── SENSOR_FUSION_SUMMARY.md           # NEW: This file
```

## Key Changes Summary

| What | Before | After |
|------|--------|-------|
| IMU Command | `z` | `i` |
| IMU Data | Yaw only (1 value) | Full 9-DOF (9 values) |
| IMU Message | Orientation only | Orientation + gyro + accel |
| Frame ID | `base_link` | `imu_link` |
| Odometry | Raw wheel slippage | EKF fused (corrected) |
| Output Topic | `/odom` only | `/odom` + `/odometry/filtered` |

---

**Status**: ✅ ROS2 code complete and ready to test once Arduino firmware is updated!
