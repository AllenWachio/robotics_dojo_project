# 🚀 Quick Launch Guide: Incremental Navigation

## Step-by-Step Launch Sequence

### Prerequisites Check ✓

Before starting, verify you have:

- ✅ Raspberry Pi running (Arduino bridge + LiDAR)
- ✅ Laptop connected to same network as Pi
- ✅ ROS2 sourced: `source ~/ros2_ws/install/setup.bash`

---

## 📋 Complete Launch Sequence

### **Step 1: Start Hardware (on Pi - already done!)**

```bash
# ON RASPBERRY PI
./01_arduino.sh    # Arduino bridge
./02_lidar.sh      # LiDAR scan
```

✅ You said you've already done this!

---

### **Step 2: Start Navigation Stack (on Laptop)**

You have **two options**:

#### **Option A: Full SLAM + Navigation (Recommended for first time)**

```bash
# ON LAPTOP
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./03_slam.sh
```

This launches:

- SLAM Toolbox (builds/loads map)
- AMCL Localization
- Nav2 Navigation Stack
- RViz visualization

**Wait for RViz to open**, then:

---

### **Step 3: Localize Robot in RViz**

This is **CRITICAL** - the robot must know where it is!

1. **In RViz**, find the toolbar at the top
2. Click **"2D Pose Estimate"** button
3. **Click on map** where robot actually is
4. **Drag** to set robot's orientation (direction it's facing)

You should see:

- Green arrow showing robot's estimated position
- Laser scan data aligning with map walls

**Verify localization:**

```bash
# In a new terminal
ros2 topic echo /amcl_pose --once
```

You should see current pose coordinates.

---

### **Step 4: Launch Incremental Navigation Test**

Now you're ready!

```bash
# ON LAPTOP (new terminal)
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./06_incremental_test.sh
```

This script will:

1. ✅ Check all prerequisites (Nav2, AMCL, LiDAR)
2. ✅ Show current robot position
3. ✅ Launch the test behavior tree
4. ✅ Execute the mission

**You'll see output like:**

```
============================================================
  TEST: Incremental Navigation with Obstacle Avoidance
============================================================

[1/4] Pre-Flight System Checks
--------------------------------------------------------------
Navigation Stack:
Checking Nav2 Action Server... ✓ Active
Checking Controller Server... ✓ Running
...

[MoveToPointA]: Starting incremental movement mission
[MoveToPointA]: Generated 4 waypoints
[MoveToPointA] ✓ Waypoint 1:
  Position: (0.127, 0.123)
  Progress: 25.3%
  ...
```

---

## 🎯 Alternative: Launch Your Own Mission

### **Option 1: Use the Test Mission**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree
export PYTHONPATH="${PYTHONPATH}:${PWD}"
python3 test_incremental_navigation.py
```

### **Option 2: Create Custom Mission**

Create `my_mission.py`:

```python
#!/usr/bin/env python3
import rclpy
import py_trees
from incremental_move_behavior import IncrementalMove

def create_my_mission():
    root = py_trees.composites.Sequence("MyMission", memory=True)

    # Add your waypoints here
    move1 = IncrementalMove("GoToA", 1.0, 1.0)
    move2 = IncrementalMove("GoToB", 2.0, -1.0)
    move3 = IncrementalMove("GoHome", 0.0, 0.0)

    root.add_children([move1, move2, move3])
    return root

def main():
    rclpy.init()
    node = rclpy.create_node('my_mission_node')

    root = create_my_mission()

    # Setup behaviors
    for behavior in py_trees.trees.BehaviourTree(root).root.iterate():
        if hasattr(behavior, 'setup'):
            behavior.setup(node=node)

    tree = py_trees.trees.BehaviourTree(root)

    rate = node.create_rate(10)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        tree.tick()

        if root.status != py_trees.common.Status.RUNNING:
            break

        try:
            rate.sleep()
        except:
            pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Then run:

```bash
chmod +x my_mission.py
python3 my_mission.py
```

---

## 🔧 Complete Terminal Layout

Here's what you should have running:

### **On Raspberry Pi:**

```
Terminal 1: Arduino Bridge (./01_arduino.sh)
Terminal 2: LiDAR        (./02_lidar.sh)
```

### **On Laptop:**

```
Terminal 1: Navigation   (./03_slam.sh) + RViz
Terminal 2: Behavior Tree (./06_incremental_test.sh)
```

---

## 📊 Visual Workflow

```
┌─────────────────────────────────────────────┐
│         RASPBERRY PI                        │
│  [Arduino Bridge] + [LiDAR Scan]            │
│         ↓ publishes topics ↓                │
└──────────────┬──────────────────────────────┘
               │ (WiFi network)
               ↓
┌─────────────────────────────────────────────┐
│         LAPTOP                              │
│                                             │
│  Step 1: Launch Navigation                 │
│  ┌─────────────────────────────────┐       │
│  │ ./03_slam.sh                    │       │
│  │  - SLAM Toolbox                 │       │
│  │  - AMCL Localization            │       │
│  │  - Nav2 Stack                   │       │
│  │  - RViz                         │       │
│  └─────────────────────────────────┘       │
│           ↓                                 │
│  Step 2: Localize in RViz                  │
│  [2D Pose Estimate] → Click on map         │
│           ↓                                 │
│  Step 3: Launch Behavior Tree              │
│  ┌─────────────────────────────────┐       │
│  │ ./06_incremental_test.sh        │       │
│  │  - Pre-flight checks            │       │
│  │  - IncrementalMove behaviors    │       │
│  │  - Obstacle avoidance           │       │
│  └─────────────────────────────────┘       │
│           ↓                                 │
│     Robot Moves! 🚀                         │
└─────────────────────────────────────────────┘
```

---

## ⚠️ Troubleshooting Launch Issues

### **Issue: "Navigation action server not available"**

**Cause:** Nav2 not running or not ready

**Solution:**

```bash
# Check if navigation is running
ros2 node list | grep controller
ros2 action list | grep navigate_to_pose

# If not found, launch navigation
./03_slam.sh
```

---

### **Issue: "Could not get robot position"**

**Cause:** AMCL not publishing or robot not localized

**Solution:**

```bash
# Check AMCL
ros2 topic echo /amcl_pose --once

# If no output, check localization
ros2 node list | grep amcl

# Localize robot in RViz using "2D Pose Estimate"
```

---

### **Issue: "LiDAR Scan not found"**

**Cause:** LiDAR not running on Pi

**Solution:**

```bash
# On Pi, check LiDAR
ros2 topic list | grep scan
ros2 topic echo /scan --once

# If not running, restart
./02_lidar.sh
```

---

### **Issue: Robot doesn't move**

**Possible causes:**

1. Not localized properly → Re-localize in RViz
2. Goal is unreachable → Test goal with "2D Goal Pose" in RViz first
3. Nav2 parameters incorrect → Check controller_server status

**Debug steps:**

```bash
# Test navigation manually in RViz
# Use "2D Goal Pose" tool to send a goal
# If that works, behavior tree should work too

# Check logs
ros2 node list
ros2 topic list
```

---

## 🎯 Quick Command Reference

### **Check System Status**

```bash
# Navigation running?
ros2 node list | grep -E "controller|planner|amcl"

# Topics publishing?
ros2 topic list | grep -E "scan|amcl_pose|cmd_vel"

# Robot position?
ros2 topic echo /amcl_pose --once
```

### **Launch Components Separately**

```bash
# SLAM only
ros2 launch slam_toolbox online_async_launch.py

# AMCL only (if you have a saved map)
ros2 launch nav2_bringup localization_launch.py map:=/path/to/map.yaml

# Nav2 only
ros2 launch nav2_bringup navigation_launch.py
```

---

## 📝 Pre-Launch Checklist

Before running `./06_incremental_test.sh`:

- [ ] Raspberry Pi powered on
- [ ] Arduino bridge running (`./01_arduino.sh`)
- [ ] LiDAR running (`./02_lidar.sh`)
- [ ] Laptop connected to network
- [ ] Navigation stack running (`./03_slam.sh`)
- [ ] RViz open and showing map
- [ ] Robot localized (green arrow visible in RViz)
- [ ] `/amcl_pose` publishing (test with `ros2 topic echo /amcl_pose --once`)
- [ ] `/scan` publishing (test with `ros2 topic echo /scan --once`)

**All checked?** → Run `./06_incremental_test.sh` 🚀

---

## 🎓 Understanding the Launch Scripts

### **`./03_slam.sh` (Navigation Stack)**

Launches:

1. **SLAM Toolbox** - Builds/loads map
2. **AMCL** - Localizes robot on map
3. **Nav2** - Navigation stack (planner + controller)
4. **RViz** - Visualization

Publishes:

- `/map` - Occupancy grid
- `/amcl_pose` - Robot position estimate

Subscribes to:

- `/scan` - LiDAR data
- `/odom` - Odometry

---

### **`./06_incremental_test.sh` (Behavior Tree)**

1. **Pre-flight checks** - Verifies all systems ready
2. **Position check** - Shows current robot location
3. **Launch** - Starts behavior tree
4. **Monitor** - Shows progress logs

Uses:

- `/amcl_pose` - For position tracking
- `/scan` - For obstacle avoidance
- `NavigateToPose` action - For movement

---

## 💡 Pro Tips

### **Tip 1: Always localize first!**

The most common mistake is forgetting to localize. The robot **must** know where it is on the map before navigation will work.

### **Tip 2: Test goals in RViz first**

Before adding waypoints to your behavior tree, test them manually:

1. Click "2D Goal Pose" in RViz
2. Click on map where you want robot to go
3. If robot navigates successfully → waypoint is good!
4. If navigation fails → choose different waypoint

### **Tip 3: Watch RViz while behavior tree runs**

You can see:

- Current waypoint goal (green arrow)
- Planned path (green line)
- LiDAR scan (red dots)
- Robot position (green/blue arrow)

### **Tip 4: Start with small movements**

The test mission uses safe coordinates:

- Point A: (0.5, 0.5)
- Point B: (0.5, -0.5)

These are close to origin (0, 0) - very safe for testing!

---

## 🚀 Ready to Launch?

### **Quick Start (After Pi is running):**

```bash
# Terminal 1 (Laptop)
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./03_slam.sh
# Wait for RViz, then localize robot with "2D Pose Estimate"

# Terminal 2 (Laptop)
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./06_incremental_test.sh
# Watch robot navigate incrementally!
```

That's it! 🎉

---

## 📚 Next Steps

1. **Test**: Run the test mission with safe waypoints
2. **Observe**: Watch logs and RViz visualization
3. **Measure**: Use coordinate converter for your game field waypoints
4. **Customize**: Edit `test_incremental_navigation.py` with your coordinates
5. **Deploy**: Use in competition! 🏆

---

**Need help? Check the logs - they're very detailed and will tell you exactly what's happening!** 🔍
