# TF Conflict Fix - Quick Reference Card

## ⚡ QUICK FIX

**Problem:** Wheels don't appear in RViz after EKF changes
**Cause:** Two nodes publishing same TF (`odom→base_link`)
**Solution:** Set `publish_tf: False` in Arduino Bridge when using EKF

---

## 🎯 THE GOLDEN RULE

**Only ONE node should publish each transform!**

When using EKF: **EKF publishes TF, Arduino Bridge does NOT**

---

## 📋 Configuration Matrix

| Launch File | Uses EKF? | Arduino `publish_tf` | Who publishes TF |
|-------------|-----------|---------------------|------------------|
| `slam_with_sensor_fusion.launch.py` | ✅ Yes | **False** | EKF |
| `arduino_bridge.py` (with sensor fusion) | ✅ Yes | **False** | EKF |
| `test_teleop.launch.py` | ❌ No | **True** | Arduino Bridge |
| `full_slam_test.launch.py` | ❌ No | **True** | Arduino Bridge |

---

## 🔧 Quick Commands

### Check TF Publishers
```bash
ros2 topic info /tf
```

### Check Arduino Bridge Config
```bash
ros2 param get /ros_arduino_bridge publish_tf
```

### Check EKF Config
```bash
ros2 param get /ekf_filter_node publish_tf
```

### Generate TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf
```

### Run Diagnostic Script
```bash
chmod +x ~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/scripts/diagnose_tf.sh
./diagnose_tf.sh
```

---

## 🌳 Correct TF Tree Structure

### With EKF (Sensor Fusion):
```
map → odom → base_link → wheels/imu/laser
      ↑               ↑
    SLAM            Robot State
    Toolbox         Publisher
      ↑
  (EKF publishes odom→base_link)
```

### Without EKF (Raw Odometry):
```
map → odom → base_link → wheels/imu/laser
      ↑               ↑
    SLAM            Robot State
    Toolbox         Publisher
      ↑
  (Arduino Bridge publishes odom→base_link)
```

---

## 🚀 Testing Steps

1. **Launch your system:**
   ```bash
   ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
   ```

2. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. **Verify in RViz:**
   - Fixed Frame: `odom` or `map`
   - Add RobotModel → wheels should appear ✅
   - Add TF → tree should be clean ✅
   - Add Map → map should form ✅

4. **Drive robot:**
   - Use teleop keyboard
   - Map should form without breaking during turns

---

## ❌ Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Both publish TF | Wheels disappear, erratic movement | Set Arduino `publish_tf: False` |
| Neither publishes TF | No robot in RViz | Enable TF in correct node |
| Wrong launch file | Mapping fails during turns | Use `slam_with_sensor_fusion.launch.py` |
| URDF not loaded | No wheels visible | Check robot_state_publisher |

---

## 🆘 Emergency Reset

If totally broken:

```bash
# 1. Kill all nodes
pkill -9 ros

# 2. Rebuild workspace
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash

# 3. Launch with sensor fusion
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py

# 4. Verify
./scripts/diagnose_tf.sh
```

---

## 📚 Related Files

- `TF_CONFLICT_FIX.md` - Detailed explanation
- `SENSOR_FUSION_IMPLEMENTATION.md` - EKF configuration
- `scripts/diagnose_tf.sh` - Automatic diagnostics
- `ekf_config.yaml` - EKF parameters

---

## ✅ Success Checklist

- [ ] Arduino Bridge: `publish_tf: False` (when using EKF)
- [ ] EKF: `publish_tf: true` (in ekf_config.yaml)
- [ ] Robot State Publisher running
- [ ] Wheels visible in RViz
- [ ] Map forms correctly
- [ ] No TF conflicts in `view_frames`

---

## 💡 Key Insights

1. **EKF adds value:** Corrects wheel slip with IMU
2. **TF is critical:** Without it, RViz can't display robot
3. **One publisher rule:** Prevents conflicts and jitter
4. **Test early:** Check TF tree before complex debugging

---

**Last Updated:** October 7, 2025
**Branch:** z_axix
