# IMU Implementation Summary

## Overview
Added IMU support to the ROS Arduino Bridge to read yaw orientation from the Arduino using serial command `z`.

## Changes Made

### 1. Publisher Added
- **Topic**: `/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **Frame ID**: `base_link`
- **Publishing Rate**: ~5Hz (via timer)

### 2. Arduino Communication
- **Command**: `z` - Request IMU yaw angle
- **Response Format**: Degrees (various formats supported)
  - `"Z:123.4"`
  - `"IMU 123.4"`
  - `"123.4"` (numeric only)

### 3. Implementation Details

#### Timer
- Added `self.imu_timer` that calls `_request_imu()` every 0.2 seconds (5Hz)
- Sends `z` command to Arduino periodically

#### Parsing Function: `_handle_imu_line()`
- Extracts yaw angle in degrees from Arduino response
- Supports multiple response formats (flexible regex matching)
- Converts degrees to radians
- Computes quaternion from yaw (roll=0, pitch=0)
- Publishes `sensor_msgs/Imu` message with orientation

#### Integration
- IMU line detection added to `update_sensor_data()`
- When reading serial responses, lines matching IMU format are automatically parsed
- IMU handling doesn't block encoder data collection

### 4. Quaternion Conversion
```python
yaw = math.radians(deg)
qz = math.sin(yaw / 2.0)
qw = math.cos(yaw / 2.0)
```
- Roll and pitch are set to 0 (x=0, y=0)
- Only yaw is used for orientation

## Testing

### On the Raspberry Pi
Build and run the updated bridge:
```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
ros2 launch ros_arduino_bridge arduino_bridge.py use_rviz:=false
```

### Monitor IMU Data
```bash
# Check if topic exists
ros2 topic list | grep imu

# Echo IMU messages
ros2 topic echo /imu/data

# Check publish rate
ros2 topic hz /imu/data

# View orientation quaternion
ros2 topic echo /imu/data --field orientation
```

### Expected Output
```
header:
  stamp:
    sec: 1759731934
    nanosec: 509277174
  frame_id: base_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.342020143326  # varies based on yaw
  w: 0.939692620786  # varies based on yaw
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
```

## Notes
- Angular velocity and linear acceleration fields are set to zero (not provided by simple yaw reading)
- If you need full 6-DOF IMU data (gyro + accel), modify Arduino to send those values and update parser
- Frame ID is set to `base_link` - adjust if your robot uses a different IMU frame
- The IMU timer runs independently of encoder polling to avoid blocking

## Future Enhancements
- Add angular velocity parsing if Arduino provides gyro data
- Add linear acceleration parsing if Arduino provides accelerometer data
- Add covariance matrices for sensor fusion (e.g., with robot_localization)
- Implement IMU calibration service
