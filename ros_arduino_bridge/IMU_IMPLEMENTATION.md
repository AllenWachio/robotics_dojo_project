# IMU Implementation Summary - UPDATED FOR FULL 9-DOF DATA

## Overview
Updated IMU support in the ROS Arduino Bridge to read **full 9-DOF IMU data** (orientation, angular velocity, linear acceleration) from the Arduino using serial command **`i`**.

## Changes Made (Latest Update)

### 1. Publisher Configuration
- **Topic**: `/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **Frame ID**: `imu_link` (matches robot_params.yaml)
- **Publishing Rate**: ~5Hz (via timer)

### 2. Arduino Communication Protocol

#### Command
**`i`** - Request full IMU data (changed from previous `z` command)

#### Response Format
The Arduino should respond with **9 space-separated floating-point values**:

```
yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z
```

**Value Definitions:**
- `yaw`, `pitch`, `roll`: Orientation angles in **degrees**
- `gyro_x`, `gyro_y`, `gyro_z`: Angular velocities in **degrees/second** (body frame)
- `accel_x`, `accel_y`, `accel_z`: Linear accelerations in **m/s²** (body frame)

**Example Arduino Response:**
```
45.30 2.10 -1.50 0.05 -0.02 12.34 0.12 -0.05 9.78
```

This represents:
- Orientation: Yaw=45.30°, Pitch=2.10°, Roll=-1.50°
- Angular Velocity: (0.05, -0.02, 12.34) deg/s
- Linear Acceleration: (0.12, -0.05, 9.78) m/s²

### 3. Implementation Details

#### Timer
```python
self.imu_timer = self.create_timer(0.2, self._request_imu)  # 5Hz
```
Sends `i` command to Arduino periodically.

#### Parsing Function: `_handle_imu_line()`
The parser:
1. Expects exactly 9 space-separated float values
2. Converts angles from degrees to radians
3. Converts angular velocities from deg/s to rad/s
4. Computes full quaternion from roll, pitch, yaw (ZYX convention)
5. Populates the complete `sensor_msgs/Imu` message:
   - `orientation`: Quaternion (x, y, z, w)
   - `angular_velocity`: (x, y, z) in rad/s
   - `linear_acceleration`: (x, y, z) in m/s²
6. Sets covariance matrices for sensor fusion

#### Quaternion Conversion
Uses ZYX Euler angle convention:
```python
# Convert angles to radians
yaw = math.radians(yaw_deg)
pitch = math.radians(pitch_deg)
roll = math.radians(roll_deg)

# Compute quaternion (ZYX convention)
cy = math.cos(yaw * 0.5)
sy = math.sin(yaw * 0.5)
cp = math.cos(pitch * 0.5)
sp = math.sin(pitch * 0.5)
cr = math.cos(roll * 0.5)
sr = math.sin(roll * 0.5)

qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy
```

#### Covariance Matrices
The implementation sets reasonable default covariances:
```python
orientation_covariance = [0.01, 0.0, 0.0,
                           0.0, 0.01, 0.0,
                           0.0, 0.0, 0.01]  # rad²

angular_velocity_covariance = [0.001, 0.0, 0.0,
                                0.0, 0.001, 0.0,
                                0.0, 0.0, 0.001]  # (rad/s)²

linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                   0.0, 0.01, 0.0,
                                   0.0, 0.0, 0.01]  # (m/s²)²
```

**Note**: These values should be tuned based on your specific IMU's datasheet specifications.

#### Integration with Serial Loop
- IMU line detection added to `update_sensor_data()`
- Lines with exactly 9 float values are automatically recognized as IMU data
- IMU handling is non-blocking and doesn't interfere with encoder data collection

## Testing

### On the Raspberry Pi

#### 1. Rebuild the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

#### 2. Launch the Bridge Node
```bash
ros2 launch ros_arduino_bridge arduino_bridge.py
```

#### 3. Check IMU Topic
In a new terminal:
```bash
# See if /imu/data is publishing
ros2 topic list | grep imu

# View IMU messages
ros2 topic echo /imu/data

# Check publishing rate (should be ~5Hz)
ros2 topic hz /imu/data
```

#### 4. Inspect IMU Data
```bash
ros2 topic echo /imu/data --once
```

**Expected Output:**
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: imu_link
orientation:
  x: 0.012  # Quaternion from roll/pitch/yaw
  y: 0.018
  z: 0.383
  w: 0.923
orientation_covariance:
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
angular_velocity:
  x: 0.0008  # rad/s (converted from deg/s)
  y: -0.0003
  z: 0.215
angular_velocity_covariance:
- 0.001
- 0.0
- 0.0
- 0.0
- 0.001
- 0.0
- 0.0
- 0.0
- 0.001
linear_acceleration:
  x: 0.12   # m/s²
  y: -0.05
  z: 9.78
linear_acceleration_covariance:
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
```

#### 5. Test Arduino Command Directly
```bash
# Send 'i' command and observe response
ros2 run ros_arduino_bridge test_serial_command.py  # If available
# OR use Arduino Serial Monitor at 57600 baud and type 'i'
```

**Expected Arduino Response:**
```
45.30 2.10 -1.50 0.05 -0.02 12.34 0.12 -0.05 9.78
```

## Sensor Fusion with robot_localization

### Configuration
The full 9-DOF IMU data enables sensor fusion with `robot_localization` EKF to correct wheel slippage:

**Config File**: `config/ekf_config.yaml`

**Key Fusion Settings:**
- **From /odom**: Linear velocities (x_vel, y_vel)
- **From /imu/data**: 
  - Orientation (yaw) - accurate heading
  - Angular velocity (yaw_vel) - corrects wheel slip during turns
  - Linear acceleration (x, y, z) - dynamics

### Launch Sensor Fusion
```bash
# Terminal 1: Arduino bridge (provides /odom and /imu/data)
ros2 launch ros_arduino_bridge arduino_bridge.py

# Terminal 2: Sensor fusion EKF
ros2 launch ros_arduino_bridge sensor_fusion.launch.py
```

### Monitor Fused Odometry
```bash
# View fused output
ros2 topic echo /odometry/filtered

# Compare raw vs fused odometry
ros2 run rqt_plot rqt_plot /odom/pose/pose/position/x /odometry/filtered/pose/pose/position/x
```

## Troubleshooting

### IMU Not Publishing
1. **Check Arduino Connection**:
   ```bash
   ros2 topic echo /diagnostics  # Look for serial errors
   ```

2. **Verify Arduino Response**:
   - Use Arduino Serial Monitor at 57600 baud
   - Type `i` and press Enter
   - Should see 9 space-separated numbers

3. **Check Node Logs**:
   ```bash
   ros2 launch ros_arduino_bridge arduino_bridge.py --log-level DEBUG
   ```
   Look for `[IMU]` messages in the output.

### Wrong Data Format
If Arduino is sending different format (e.g., single yaw value):
- Update Arduino firmware to send full 9-value format
- See `NEW_SENSORS_IMPLEMENTATION.md` for Arduino code requirements

### Covariance Tuning
If EKF behaves strangely:
1. Check IMU covariances in `_handle_imu_line()` match your sensor specs
2. Adjust EKF process noise in `ekf_config.yaml`
3. Compare raw topics: `ros2 topic echo /odom` vs `ros2 topic echo /imu/data`

### Frame ID Mismatch
If TF errors appear:
- Verify `imu_frame: "imu_link"` in `config/robot_params.yaml`
- Check URDF defines `imu_link` frame
- Ensure `_handle_imu_line()` uses `frame_id = 'imu_link'`

## Future Enhancements

### Covariance Tuning
- [ ] Tune covariance values based on actual MPU6050/MPU9250 datasheet specs
- [ ] Add dynamic covariance adjustment based on motion state
- [ ] Implement covariance validation checks

### Additional Sensors
- [ ] Add magnetometer data (if available) for absolute heading
- [ ] Integrate GPS for outdoor localization
- [ ] Add visual odometry fusion

### Advanced Filtering
- [ ] Switch to UKF for better non-linear motion handling
- [ ] Add dual-EKF setup (local + global frames)
- [ ] Implement adaptive filtering for variable terrain

## References
- [sensor_msgs/Imu Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)
- [robot_localization Documentation](http://docs.ros.org/en/humble/p/robot_localization/)
- [MPU6050 DMP Quaternion Output](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Quaternion to Euler Conversion](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
