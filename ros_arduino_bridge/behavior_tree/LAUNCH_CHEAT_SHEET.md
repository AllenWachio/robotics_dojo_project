# ⚡ LAUNCH CHEAT SHEET - Incremental Navigation

## 🎯 After Pi is Running (Arduino + LiDAR)

### **Terminal 1 (Laptop): Navigation Stack**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./02c_slam_navigation_mode.sh
```

**Wait for RViz to open** → Then **LOCALIZE ROBOT**:

1. Click **"2D Pose Estimate"** in RViz toolbar
2. Click on map where robot **actually is**
3. Drag to set robot's **orientation**

Verify:

```bash
ros2 topic echo /amcl_pose --once
```

---

### **Terminal 2 (Laptop): Behavior Tree**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./06_incremental_test.sh
```

---

## ✅ Quick Checklist

- [ ] Pi running (Arduino + LiDAR)
- [ ] Navigation launched (Terminal 1)
- [ ] RViz showing map
- [ ] Robot localized (green arrow visible)
- [ ] `/amcl_pose` publishing
- [ ] Ready to launch behavior tree!

---

## 🚨 Common Issues

| Problem              | Solution                                 |
| -------------------- | ---------------------------------------- |
| "Nav2 not available" | Launch navigation first (Terminal 1)     |
| "No AMCL pose"       | Localize robot in RViz                   |
| Robot doesn't move   | Test with "2D Goal Pose" in RViz first   |
| "LiDAR not found"    | Check Pi: `ros2 topic echo /scan --once` |

---

## 📚 Full Guides

- `HOW_TO_LAUNCH.sh` - Run this script for visual guide
- `LAUNCH_GUIDE.md` - Complete step-by-step instructions
- `QUICK_REFERENCE.md` - IncrementalMove API reference

---

## 🎮 Quick Test Commands

```bash
# Show help
./HOW_TO_LAUNCH.sh

# Check system status
ros2 node list | grep -E "controller|planner|amcl"
ros2 topic list | grep -E "scan|amcl|cmd_vel"

# Get robot position
ros2 topic echo /amcl_pose --once

# Test navigation in RViz
# Use "2D Goal Pose" tool → Click on map
```

---

**That's it! Two terminals on laptop, and you're navigating! 🚀**
