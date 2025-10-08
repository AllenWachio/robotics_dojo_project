# 🏁 COMPETITION DAY CHEAT SHEET

## ⚡ Start Mission (Copy-Paste Ready)

```bash
# === TERMINAL 1: Robot Core ===
ros2 launch ros_arduino_bridge full_slam_test.launch.py

# === TERMINAL 2: Navigation ===
ros2 launch ros_arduino_bridge laptop_navigation.launch.py

# === TERMINAL 3: Camera ===
ros2 run rpi_camera_package color_detection_node

# === TERMINAL 4: Disease Detection ===
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node

# === TERMINAL 5: Mission ===
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree
python3 launch_competition.py
```

## 🛑 EMERGENCY STOP

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0}, angular: {z: 0}}"
```

## ✅ Pre-Flight Checks

```bash
# Verify all systems
python3 launch_competition.py --dry-run

# Quick test
python3 test_behaviors.py all
```

## 📊 Expected Performance

- Disease Detection: **20-30s**
- Cargo Loading: **15-20s**
- Maze Navigation: **40-60s**
- Cargo Delivery: **20-30s**
- **TOTAL: 95-140s (1.5-2.3 min)** ✅

## 🎯 Key Files

- **Main Mission**: `competition_mission.py`
- **Testing**: `test_behaviors.py`
- **Docs**: `README_COMPETITION.md`
- **Visual Guide**: `VISUAL_OVERVIEW.md`

## Good Luck! 🏆
