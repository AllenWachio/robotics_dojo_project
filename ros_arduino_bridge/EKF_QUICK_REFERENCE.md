# EKF Sensor Fusion - Quick Reference

## 🚀 Quick Start Commands

### Launch Everything (Recommended)

```bash
ros2 launch ros_arduino_bridge arduino_bridge_with_ekf.launch.py
```

### Launch Separately

```bash
# Terminal 1
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false

# Terminal 2
ros2 launch ros_arduino_bridge sensor_fusion.launch.py
```

### Standalone (No EKF)

```bash
ros2 launch ros_arduino_bridge arduino_bridge.py
```

---

## 🔍 Quick Checks

### Check Covariance

```bash
ros2 topic echo /odom --once | grep -A 36 covariance
# Should see non-zero values
```

### Check IMU

```bash
ros2 topic echo /imu/data --once
# frame_id should be: base_link
```

### Check Topics

```bash
ros2 topic list | grep odom
# Should see: /odom and /odometry/filtered (when EKF running)
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf - should be clean (no warnings)
```

### Check Publishing Rates

```bash
ros2 topic hz /odom              # ~10Hz
ros2 topic hz /odometry/filtered # ~30Hz
ros2 topic hz /imu/data          # ~5Hz
```

---

## 🎯 Test Sequence

1. **Verify covariance** → `ros2 topic echo /odom --once`
2. **Verify IMU frame** → `ros2 topic echo /imu/data --once`
3. **Launch EKF** → `ros2 launch ros_arduino_bridge arduino_bridge_with_ekf.launch.py`
4. **Check topics** → `ros2 topic list | grep odom`
5. **Check TF** → `ros2 run tf2_tools view_frames`
6. **Drive robot** → Test in square pattern
7. **Compare odometry** → Raw vs filtered

---

## ⚙️ Configuration Parameters

### Arduino Bridge

```bash
ros2 launch ros_arduino_bridge arduino_bridge.py \
  serial_port:=/dev/ttyUSB0 \
  publish_tf:=false \
  baud_rate:=57600
```

### Key Parameters

- `publish_tf` - Set to `false` when using EKF
- `base_width` - Distance between wheels (affects rotation)
- `wheel_radius` - Wheel radius (affects velocity)
- `encoder_ticks_per_rev` - Encoder resolution

---

## 🔧 Troubleshooting

| Problem               | Solution                                                  |
| --------------------- | --------------------------------------------------------- |
| No /odometry/filtered | Install: `sudo apt install ros-humble-robot-localization` |
| TF warnings           | Use `publish_tf:=false` on bridge                         |
| All-zero covariance   | Rebuild package                                           |
| IMU shows zeros       | Check Arduino response to 'i' command                     |
| Robot jumps           | Check TF tree for multiple publishers                     |

---

## 📊 Expected Results

### Odometry Covariance Values

```
pose.covariance[0]  = 0.01  # x position (low = good)
pose.covariance[7]  = 0.01  # y position (low = good)
pose.covariance[35] = 0.5   # yaw (high = trust IMU instead)
```

### Topics When Running

- `/odom` - Raw encoder odometry
- `/imu/data` - IMU sensor data
- `/odometry/filtered` - Fused odometry (EKF output)

### TF Tree

```
map
 └─ odom (published by EKF)
     └─ base_link
         └─ [other robot links]
```

---

## 📞 Need Help?

1. Check logs: `ros2 launch ... --log-level DEBUG`
2. Verify Arduino: Send 'i' command, expect 9 values
3. Check covariance: Must be non-zero
4. Verify TF: Only ONE publisher for odom→base_link

---

## 🎓 Key Concepts

**Covariance:** How much to trust a sensor

- Low value (0.01) = High trust
- High value (0.5) = Low trust

**Why use EKF:**

- Encoders slip during turns → bad rotation
- IMU doesn't slip → good rotation
- EKF combines both → best estimate

**TF Publishing:**

- Only ONE node should publish odom→base_link
- When using EKF: Bridge TF OFF, EKF TF ON
- Without EKF: Bridge TF ON

---

Created: 2025-10-06
Status: ✅ Production Ready
