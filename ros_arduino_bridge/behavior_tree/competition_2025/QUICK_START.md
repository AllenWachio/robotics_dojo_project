# ⚡ QUICK START - Competition Day

## 🎯 3-Step Launch (< 2 minutes)

### On Raspberry Pi (Terminal 1):

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025/launch
./01_pi_hardware.sh
```

✅ **Leave running!**

---

### On Laptop (Terminal 1):

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025/launch
./02_laptop_processing.sh
```

⚠️ **Set initial pose in RViz** (2D Pose Estimate tool)
✅ **Leave running!**

---

### On Laptop (Terminal 2):

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025/launch
./03_run_mission.sh
```

🤖 **Robot is now AUTONOMOUS!**

---

## 📋 Pre-Competition Checklist

**Pi Hardware:**

- [ ] Arduino plugged in
- [ ] LiDAR plugged in
- [ ] Camera connected
- [ ] Battery charged
- [ ] Pi powered on
- [ ] SSH accessible

**Laptop:**

- [ ] Competition map: `~/ros2_ws/maps/gamefield.yaml`
- [ ] Laptop on same network as Pi
- [ ] ROS_DOMAIN_ID matches Pi
- [ ] py_trees installed: `pip3 install py_trees py_trees_ros`

---

## 🚨 Emergency Commands

**Stop robot:**

```bash
Ctrl+C  (in mission terminal)
```

**Emergency teleop:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Check if Nav2 ready:**

```bash
ros2 node list | grep amcl
```

**Check Pi topics:**

```bash
ros2 topic list | grep odom
```

**Restart if stuck:**

1. Ctrl+C all terminals
2. Wait 5 seconds
3. Restart from Step 1

---

## 🎬 Mission Phases (Automatic)

1. **Disease Detection** (optional)

   - Navigate to plant
   - Run ML inference
   - Return home

2. **Cargo Loading**

   - Navigate to loading bay
   - Reverse in
   - Read RGB sensor color
   - Exit bay

3. **Maze Navigation**

   - Navigate to delivery zone (color-based)
   - Monitor camera
   - Avoid obstacles

4. **Cargo Delivery**
   - Reverse into bay
   - Activate conveyor
   - Mission complete!

---

## 🔍 Quick Diagnostics

```bash
# See what's running
ros2 node list

# Check robot position
ros2 topic echo /amcl_pose --once

# Check if robot receives commands
ros2 topic echo /cmd_vel

# Check camera feed
ros2 topic hz /camera/image_raw/compressed

# Check color sensor
ros2 topic echo /color_sensor/rgb --once
```

---

## 💡 Tips

- **RViz freezes**: Reduce particle count (AMCL)
- **Robot stuck**: Check LiDAR data in RViz
- **Wrong delivery zone**: Check cargo color detection
- **Slow navigation**: Increase max velocities in nav2_params
- **ML not detecting**: Check camera feed compression

---

## 📁 Important Files

- **Mission code**: `competition_2025/competition_mission.py`
- **Behaviors**: `competition_2025/behaviors/`
- **Waypoints**: Edit `WAYPOINTS = {}` in `competition_mission.py`
- **Nav2 config**: `ros_arduino_bridge/config/nav2_params.yaml`

---

## 🏆 Competition Flow

```
Setup (2 min)
    ↓
01_pi_hardware.sh (on Pi)
    ↓
02_laptop_processing.sh (on Laptop)
    ↓
Set initial pose (RViz)
    ↓
03_run_mission.sh (on Laptop)
    ↓
HANDS OFF! 👐
    ↓
Watch robot work 🤖
    ↓
Mission complete! 🎉
    ↓
Time: ~2.5-3 min
```

---

**Questions? See `DEPLOYMENT_GUIDE.md` for full details!**
