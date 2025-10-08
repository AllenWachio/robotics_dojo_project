# Navigation Troubleshooting Guide

## Complete Command Reference for Diagnosing Navigation Issues

This guide provides **all the commands** you need to verify your navigation system is working correctly and identify problems early, **especially the critical issue where Nav2 Goal doesn't publish to /cmd_vel**.

---

## 🚨 Quick Start: Run Diagnostics Script

**The easiest way to check everything:**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./05_navigation_diagnostics.sh
```

This automated script checks:

- ✅ All Nav2 nodes are running
- ✅ TF transforms are working
- ✅ Topics are publishing data
- ✅ /cmd_vel connectivity
- ✅ Costmaps are active
- ✅ Localization status

**Run this BEFORE trying to send navigation goals!**

---

## 📋 Manual Diagnostic Commands

If you want to check specific things manually, use these commands:

### **1. Check if Nav2 Nodes Are Running**

**The Problem:** Nav2 Goal tool won't work if navigation nodes aren't running.

```bash
# Check for all Nav2 core nodes
ros2 node list | grep -E "(controller|planner|bt_navigator|behavior)"

# Expected output:
# /behavior_server
# /bt_navigator
# /controller_server
# /planner_server
```

**If missing:**

- You didn't launch the navigation script!
- Run: `./02c_slam_navigation_mode.sh` or `./02_navigation_mode.sh`

---

### **2. Verify TF Transform Chain**

**The Problem:** Navigation needs complete transform chain: map → odom → base_link

```bash
# Check map -> odom (localization provides this)
ros2 run tf2_ros tf2_echo map odom

# Expected: Shows translation and rotation values
# If error "frame does not exist": Localization node not running!

# Check odom -> base_link (odometry provides this)
ros2 run tf2_ros tf2_echo odom base_link

# Expected: Shows translation and rotation values
# If error: Robot hardware not publishing odometry!

# Check full chain
ros2 run tf2_ros tf2_echo map base_link

# Expected: Shows values (combines above two)
```

**Quick check all frames:**

```bash
ros2 run tf2_tools view_frames.py
# Wait 5 seconds, then check: frames_YYYY-MM-DD_HH.MM.SS.pdf
# Should show: map -> odom -> base_link -> ... (other links)
```

**If TF broken:**

- Missing map frame: No localization (AMCL or SLAM Toolbox)
- Missing odom: Robot hardware not publishing odometry
- Use: `ros2 node list` to see what's running

---

### **3. Verify Critical Topics Exist**

**The Problem:** Nav2 Goal needs these topics to work.

```bash
# Check all critical topics at once
ros2 topic list | grep -E "(goal_pose|cmd_vel|scan|odom|map|plan)"

# Expected output:
# /cmd_vel              ← Robot velocity commands (CRITICAL!)
# /goal_pose            ← Navigation goals
# /map                  ← Map data
# /odom                 ← Odometry
# /plan                 ← Global path
# /scan                 ← LiDAR data
```

**Check specific topic:**

```bash
ros2 topic info /cmd_vel

# Expected output:
# Type: geometry_msgs/msg/Twist
# Publisher count: 1 (controller_server)
# Subscription count: 1 or more (your robot)
```

**If /cmd_vel has 0 subscribers:**

- ⚠️ **CRITICAL ISSUE!** Robot isn't listening to velocity commands!
- Check if robot hardware is running on Pi
- Check: `ros2 node list` for `/ros_arduino_bridge`

---

### **4. Test /cmd_vel Publishing (Critical!)**

**The Problem:** This is THE issue you had - Nav2 Goal sets goals but robot doesn't move because /cmd_vel isn't working.

```bash
# Step 1: Check topic info
ros2 topic info /cmd_vel

# Step 2: Monitor cmd_vel in real-time
ros2 topic echo /cmd_vel

# Step 3: Test manual velocity command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Robot should move forward slightly!
# If not: Problem with robot hardware receiving commands
```

**Advanced: Check message flow rate**

```bash
# How fast is controller publishing?
ros2 topic hz /cmd_vel

# Expected when navigating: ~20 Hz (based on your controller_frequency)
# If 0 Hz when goal is active: Controller isn't generating commands!
```

**If /cmd_vel not working:**

1. Check subscriber count: `ros2 topic info /cmd_vel`
2. If 0 subscribers: Robot hardware not connected
3. Verify on Pi: `ros2 topic list | grep cmd_vel`
4. Check network: Both devices on same WiFi and ROS_DOMAIN_ID

---

### **5. Verify Localization is Active**

**The Problem:** Nav2 needs to know where robot is on the map.

**For AMCL:**

```bash
# Check if AMCL is running
ros2 node list | grep amcl

# Check particle cloud (localization quality)
ros2 topic echo /particle_cloud --once

# Check pose with covariance
ros2 topic echo /amcl_pose --once

# Covariance values should be small (< 0.5) for good localization
```

**For SLAM Toolbox:**

```bash
# Check if SLAM Toolbox is running
ros2 node list | grep slam_toolbox

# Check if it's publishing map->odom transform
ros2 run tf2_ros tf2_echo map odom

# Should show valid transform
```

**Set initial pose if not localized:**

- In RViz, click "2D Pose Estimate"
- Click on map where robot is
- Drag to set orientation
- Wait 3-5 seconds

---

### **6. Check Costmaps Are Publishing**

**The Problem:** Planner needs costmaps to plan paths around obstacles.

```bash
# Check global costmap
ros2 topic echo /global_costmap/costmap --once

# Check local costmap
ros2 topic echo /local_costmap/costmap --once

# Check costmap updates
ros2 topic hz /global_costmap/costmap_updates
ros2 topic hz /local_costmap/costmap_updates

# Expected: ~1 Hz for global, ~5 Hz for local
```

**Visualize in RViz:**

- Add display: `/global_costmap/costmap` (type: Costmap)
- Add display: `/local_costmap/costmap` (type: Costmap)
- Should see colored grid around obstacles

---

### **7. Verify Lifecycle States**

**The Problem:** Nav2 nodes must be in "active" state to work.

```bash
# List all lifecycle nodes
ros2 lifecycle nodes

# Check state of each node
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server

# All should be: "active [3]"
# If "inactive" or "unconfigured": Lifecycle manager didn't activate them
```

**Manually activate if needed:**

```bash
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /planner_server activate
# etc...

# Or restart navigation launch file with autostart:=true
```

---

### **8. Test Complete Navigation Pipeline**

**Step-by-step verification of the full navigation flow:**

#### **Step 1: Verify sensor inputs**

```bash
# LiDAR data
ros2 topic hz /scan
# Expected: ~5-10 Hz

# Odometry
ros2 topic hz /odom
# Expected: ~30-50 Hz

# Map
ros2 topic echo /map --once
# Should show map data
```

#### **Step 2: Verify localization**

```bash
# Check map->odom transform exists
ros2 run tf2_ros tf2_echo map odom

# Should show valid numbers, not "frame does not exist"
```

#### **Step 3: Send test goal programmatically**

```bash
# Send a goal near robot's current position
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, \
   pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# Then monitor what happens:
```

#### **Step 4: Watch navigation response**

```bash
# In separate terminals:

# Terminal 1: Watch for path planning
ros2 topic echo /plan

# Terminal 2: Watch for velocity commands
ros2 topic echo /cmd_vel

# Terminal 3: Watch for status
ros2 topic echo /bt_navigator/transition_event

# If goal accepted: You should see data in all three terminals
# If nothing happens: Check logs for errors
```

---

### **9. Check for Error Messages**

**View logs from each node:**

```bash
# Controller errors
ros2 topic echo /controller_server/transition_event

# Planner errors
ros2 topic echo /planner_server/transition_event

# BT Navigator status
ros2 topic echo /bt_navigator/transition_event

# System-wide diagnostics
ros2 topic echo /diagnostics
```

**View node logs directly:**

```bash
# See all log output
ros2 run rqt_console rqt_console

# Or watch specific node output when launching:
ros2 launch ... --log-level DEBUG
```

---

### **10. Network Connectivity (Multi-machine setup)**

**The Problem:** Laptop can't communicate with Pi.

```bash
# On both laptop and Pi, check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID
# Should be same on both (e.g., 0 or empty)

# Check if you can see Pi's nodes from laptop
ros2 node list | grep arduino

# Expected: /ros_arduino_bridge (from Pi)

# Check if topics from Pi are visible
ros2 topic list | grep -E "(odom|scan)"

# Expected: /odom and /scan

# Test latency
ros2 topic hz /odom
# Should show Hz value, not timeout
```

**If no connectivity:**

```bash
# Check network on both machines
ip addr show

# Check ROS 2 discovery
export ROS_DOMAIN_ID=0  # Set to same value
# Then restart nodes
```

---

## 🔧 Common Issues and Solutions

### **Issue 1: "Can't set Nav2 Goal in RViz"**

**Symptoms:** Nav2 Goal tool is grayed out or does nothing

**Diagnosis:**

```bash
ros2 topic list | grep goal_pose
# If missing: Nav2 not running!

ros2 node list | grep bt_navigator
# If missing: Navigation stack not launched
```

**Solution:**

- Launch navigation: `./02c_slam_navigation_mode.sh`
- Make sure RViz config loads Nav2 plugins

---

### **Issue 2: "Goal accepted but robot doesn't move"**

**Symptoms:** Can set goal, path shows in RViz, but robot stays still

**Diagnosis:**

```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Check if robot is subscribed
ros2 topic info /cmd_vel
# Subscription count should be > 0

# Check controller status
ros2 lifecycle get /controller_server
# Should be "active"
```

**Solution:**

- If no /cmd_vel data: Check controller_server logs
- If no subscribers: Robot hardware not running
- If inactive: Lifecycle manager issue

---

### **Issue 3: "cmd_vel publishes but robot doesn't move"**

**Symptoms:** See velocity commands but robot doesn't respond

**Diagnosis:**

```bash
# Test manual command
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# If robot doesn't move: Hardware problem
# If robot moves: Navigation configuration problem
```

**Solution:**

- Check arduino_bridge is running on Pi
- Check motor drivers are enabled
- Verify /cmd_vel subscriber on Pi: `ros2 node info /ros_arduino_bridge`

---

### **Issue 4: "No path generated"**

**Symptoms:** Goal accepted but no green path line in RViz

**Diagnosis:**

```bash
# Check planner status
ros2 lifecycle get /planner_server

# Check if map is loaded
ros2 topic echo /map --once

# Check goal is reachable
ros2 topic echo /global_costmap/costmap --once
# Goal shouldn't be in an occupied area
```

**Solution:**

- Set goal in free space (gray area in RViz)
- Check map is loaded properly
- Verify planner_server is active

---

### **Issue 5: "Map frame doesn't exist"**

**Symptoms:** TF errors about missing "map" frame

**Diagnosis:**

```bash
# Check if localization is running
ros2 node list | grep -E "(amcl|slam_toolbox)"

# Check map server
ros2 node list | grep map_server

# Check map topic
ros2 topic list | grep /map
```

**Solution:**

- Launch navigation mode, not just robot hardware
- Ensure map file exists in `~/ros2_ws/maps/`
- Check localization node is providing map->odom transform

---

## 📊 Success Checklist

Before trying to navigate, verify ALL of these:

- [ ] **Pi Arduino bridge running** (`ros2 node list | grep arduino`)
- [ ] **Pi LiDAR running** (`ros2 topic echo /scan --once`)
- [ ] **Laptop navigation launched** (`ros2 node list | grep -E "planner|controller"`)
- [ ] **TF map->odom exists** (`ros2 run tf2_ros tf2_echo map odom`)
- [ ] **TF odom->base_link exists** (`ros2 run tf2_ros tf2_echo odom base_link`)
- [ ] **/cmd_vel has subscribers** (`ros2 topic info /cmd_vel`)
- [ ] **Map is loaded** (`ros2 topic echo /map --once`)
- [ ] **Initial pose set** (Use "2D Pose Estimate" in RViz)
- [ ] **Localization converged** (Wait 3-5 seconds after setting pose)
- [ ] **Nav2 Goal tool available** (Check RViz toolbar)

**If ALL checked: You're ready to navigate!** 🎉

---

## 🎯 Quick Test Procedure

**Complete test in under 2 minutes:**

```bash
# 1. Run diagnostics
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./05_navigation_diagnostics.sh

# 2. If all checks pass, test manual goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, \
   pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
          orientation: {w: 1.0}}}"

# 3. Watch for response
ros2 topic echo /cmd_vel --once

# If you see velocity commands: SUCCESS! ✅
# If nothing: Check diagnostics output for issues
```

---

## 📞 Getting Help

If you're still stuck after checking everything:

1. **Run diagnostics and save output:**

   ```bash
   ./05_navigation_diagnostics.sh > nav_diagnostics.txt
   ```

2. **Capture node graph:**

   ```bash
   ros2 run rqt_graph rqt_graph
   # Save screenshot
   ```

3. **Export TF tree:**

   ```bash
   ros2 run tf2_tools view_frames.py
   # Creates PDF showing transform tree
   ```

4. **Check logs:**
   ```bash
   ros2 log | grep -E "ERROR|WARN"
   ```

Share these files when asking for help!

---

## 🎓 Understanding the Flow

**Here's what SHOULD happen when you set a Nav2 Goal:**

1. **RViz Nav2 Goal tool** → Publishes to `/goal_pose`
2. **bt_navigator** → Receives goal, starts behavior tree
3. **planner_server** → Creates global path, publishes to `/plan`
4. **controller_server** → Follows path, publishes to `/cmd_vel`
5. **ros_arduino_bridge (Pi)** → Receives `/cmd_vel`, drives motors
6. **Robot moves!** 🤖

**Each step depends on the previous one. If ANY step fails, robot won't move.**

Use the diagnostic commands above to find which step is failing!

---

## ✨ Pro Tips

1. **Always run diagnostics first** - Saves debugging time
2. **Test manual /cmd_vel** - Confirms hardware connection
3. **Watch /plan topic** - Shows if planner is working
4. **Monitor TF continuously** - Catches transform issues early
5. **Use RViz displays** - Visualize costmaps and paths
6. **Start with short goals** - Test basic functionality first
7. **Check logs in rqt_console** - See detailed error messages

---

## 🚀 Quick Reference Commands

```bash
# Complete system check
./05_navigation_diagnostics.sh

# Test /cmd_vel works
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Check Nav2 nodes
ros2 node list | grep -E "controller|planner|bt_navigator"

# Check TF chain
ros2 run tf2_ros tf2_echo map base_link

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check for errors
ros2 topic echo /diagnostics | grep -i error

# View full node graph
ros2 run rqt_graph rqt_graph
```

---

**Remember:** The diagnostics script (`05_navigation_diagnostics.sh`) runs all these checks automatically. Use it before every navigation session! 🎯
