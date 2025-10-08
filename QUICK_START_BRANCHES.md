# 🎯 QUICK REFERENCE - Branch Usage

## Current Situation
✅ **`merge_All` branch created and pushed!**
✅ Contains BOTH sensor fusion AND behavior tree work
✅ Ready for production testing

---

## Quick Commands

### Switch to Unified Branch (RECOMMENDED):
```bash
git checkout merge_All
```

### Test Complete System:
```bash
# On Raspberry Pi - Hardware
ros2 launch ros_arduino_bridge deployment/pi/launch/pi_robot_hardware.launch.py

# On Laptop - Behavior Tree Mission
ros2 launch ros_arduino_bridge behavior_tree_navigation.launch.py

# On Laptop - SLAM with Sensor Fusion
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
```

---

## What's in `merge_All`?

### FROM `z_axix` (Sensor Fusion):
- ✅ Drift-free odometry (EKF fusion)
- ✅ IMU 9-DOF integration
- ✅ Fixed TF conflicts
- ✅ Motor debugging (M4 issue)
- ✅ 15+ documentation files

### FROM `first-pytrees` (Behavior Tree):
- ✅ Py_trees mission orchestration
- ✅ Disease detection with timeout
- ✅ Incremental navigation
- ✅ Cube delivery mission
- ✅ 25+ documentation files

### FROM `making_pytrees_work_well` (Latest Py_trees Work):
- ✅ `competition_2025/` directory with modular behaviors
- ✅ Organized `nodes/` directory structure
- ✅ Competition mission sequence
- ✅ 13+ new documentation files
- ✅ Test and visualization tools

### NEW in `merge_All`:
- ✅ Unified launch files
- ✅ Proper TF handling for all scenarios
- ✅ Complete documentation (53+ files total!)
- ✅ Both sensor fusion AND py_trees competition work
- ✅ 46 files changed, 11,087 lines added
- ✅ Production-ready configuration

---

## Branch Decision Tree

```
Need behavior trees? ──┐
                       ├─ YES ─┐
Need sensor fusion? ───┤       ├─→ Use merge_All ✅
                       ├─ NO  ─┘
                       │
                       ├─ YES ─→ Use z_axix
                       └─ NO  ─→ Use first-pytrees
```

**Simple Answer: Just use `merge_All`!** 🚀

---

## Launch File Quick Guide

| Launch File | Branch | Purpose | publish_tf |
|-------------|--------|---------|------------|
| `slam_with_sensor_fusion.launch.py` | merge_All, z_axix | SLAM + EKF (drift-free) | False |
| `behavior_tree_navigation.launch.py` | merge_All, first-pytrees | Full mission | Varies |
| `arduino_bridge.py` | All | Basic bridge | Configurable |
| `test_teleop.launch.py` | All | Manual testing | True |

---

## Verify Your Setup

### Check Current Branch:
```bash
git branch
# You should see: * merge_All
```

### Check Sensor Fusion Files:
```bash
ls ros_arduino_bridge/*.md | grep -E "SENSOR|EKF|IMU|DRIFT"
# Should show sensor fusion documentation
```

### Check Behavior Tree Files:
```bash
ls ros_arduino_bridge/behavior_tree/*.py
# Should show behavior tree scripts
```

---

## Common Tasks

### Start from Scratch:
```bash
git checkout merge_All
git pull origin merge_All
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

### Run SLAM with Drift-Free Odometry:
```bash
# Pi
ros2 launch ros_arduino_bridge deployment/pi/launch/pi_robot_hardware.launch.py

# Laptop
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
```

### Run Behavior Tree Mission:
```bash
# Ensure SLAM is running first, then:
ros2 launch ros_arduino_bridge behavior_tree_navigation.launch.py
```

---

## Troubleshooting

### "Which branch am I on?":
```bash
git branch
# * indicates current branch
```

### "How do I get latest merge_All?":
```bash
git checkout merge_All
git pull origin merge_All
```

### "Wheels not showing in RViz?":
→ Check you're using correct launch file
→ `slam_with_sensor_fusion.launch.py` has correct TF settings

### "Map still breaks during turns?":
→ Make sure you're on `merge_All` or `z_axix` branch
→ Use `slam_with_sensor_fusion.launch.py`, NOT `slam_launch.py`

---

## Success Checklist

Before deployment, verify:
- [ ] On `merge_All` branch
- [ ] Code compiled successfully (`colcon build`)
- [ ] Sensor fusion documentation present (check `ros_arduino_bridge/SENSOR_FUSION_*.md`)
- [ ] Behavior tree files present (check `ros_arduino_bridge/behavior_tree/`)
- [ ] Launch files have correct `publish_tf` settings
- [ ] IMU calibration completed on startup
- [ ] Test with teleop first, then autonomous

---

## Key Files to Check

### Critical for Sensor Fusion:
- `ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py` (IMU + motor code)
- `ros_arduino_bridge/config/ekf_config.yaml` (EKF configuration)
- `ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py` (main SLAM launch)

### Critical for Behavior Tree:
- `ros_arduino_bridge/behavior_tree/robot_navigation_bt.py` (main BT)
- `ros_arduino_bridge/behavior_tree/sensor_behaviors.py` (sensor actions)
- `ros_arduino_bridge/launch/behavior_tree_navigation.launch.py` (BT launch)

---

## Remote Repository

**GitHub:** https://github.com/AllenWachio/robotics_dojo_project

**Branches Available:**
- `main` - stable release
- `merge_All` - **NEW!** unified branch (use this!)
- `z_axix` - sensor fusion only
- `first-pytrees` - behavior tree only
- `making_pytrees_work_well` - WIP behavior tree fixes

---

## Next Steps

1. **Test on hardware:**
   ```bash
   git checkout merge_All
   # Deploy to Pi and test SLAM + behavior tree
   ```

2. **If successful:**
   ```bash
   # Merge into main
   git checkout main
   git merge merge_All
   git push origin main
   ```

3. **If issues found:**
   - Fix on `merge_All` branch
   - Commit and push
   - Test again

---

**TL;DR:** Use `merge_All` branch - it has everything! 🎉

*Last Updated: October 8, 2025*
