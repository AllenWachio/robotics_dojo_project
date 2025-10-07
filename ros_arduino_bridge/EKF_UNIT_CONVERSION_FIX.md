# EKF Sensor Fusion - Unit Conversion Fix

## Problem Identified

The Arduino was sending **RAW MPU6050 register values** for gyroscope and accelerometer, but the ROS bridge was treating them as if they were already in SI units. This caused:

- **Gyro values ~131x too large**: Raw value `23.0` treated as `23 rad/s` instead of `0.00307 rad/s`
- **Accel values ~16384x too large**: Raw value `512.0` treated as `512 m/s²` instead of `0.306 m/s²`
- **Result**: Massive IMU drift causing map distortion and oscillation

## Solution Applied

### 1. Updated IMU Parser (`ros_arduino_bridge.py`)

Added proper unit conversions:

```python
# Gyroscope: RAW → deg/s → rad/s
gyro_scale = 131.0  # MPU6050 ±250°/s range
gyro_deg_s = gyro_raw / 131.0
gyro_rad_s = gyro_deg_s * (π/180)

# Accelerometer: RAW → g → m/s²
accel_scale = 16384.0  # MPU6050 ±2g range
accel_g = accel_raw / 16384.0
accel_ms2 = accel_g * 9.80665
```

### 2. Updated EKF Config (`ekf_config.yaml`)

**Key Changes**:
- Enabled full 9-DOF IMU (orientation + gyro + accel)
- Added wheel odometry position reference (prevents unbounded drift)
- Tuned covariances for noisy MPU6050
- Enabled gravity removal from accelerometer
- Increased frequency to 20Hz

**Fusion Strategy**:
- **Wheels**: Provide position reference + linear velocity
- **IMU DMP**: Provides stable orientation (yaw/pitch/roll)
- **IMU Gyro**: Corrects rotation during wheel slip
- **IMU Accel**: Helps with transient motion (with high noise tolerance)

## Testing

```bash
# Verify IMU units are correct
ros2 topic echo /imu/data --once

# Expected when stationary:
# - angular_velocity.z < 0.05 rad/s
# - linear_acceleration.z < 1.0 m/s²

# Test sensor fusion
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Compare raw vs filtered
ros2 run rqt_plot rqt_plot \
  /odom/pose/pose/orientation/z \
  /odometry/filtered/pose/pose/orientation/z
```

## Tuning

If map still drifts, **increase IMU covariances** in `ekf_config.yaml`:

```yaml
# Trust IMU less (reduce its influence)
imu0_orientation_covariance: [0.2, ..., 0.1]  # Was [0.1, ..., 0.05]
imu0_angular_velocity_covariance: [0.05, ...]  # Was [0.02, ...]
```

## Files Modified

1. `ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py`
   - Updated `_handle_imu_line()` with proper unit conversions

2. `ros_arduino_bridge/config/ekf_config.yaml`
   - Enabled full 9-DOF IMU fusion
   - Added position reference from wheels
   - Tuned covariances for MPU6050 noise characteristics

## Verification Checklist

- [ ] Gyro < 0.1 rad/s when robot stationary
- [ ] Accel < 1.0 m/s² when stationary (after gravity removal)
- [ ] Map points don't move when robot rotates in place
- [ ] Filtered odometry smoother than raw during turns
- [ ] No EKF warnings in logs

See `EKF_IMPLEMENTATION_REVIEW.md` for complete documentation.
