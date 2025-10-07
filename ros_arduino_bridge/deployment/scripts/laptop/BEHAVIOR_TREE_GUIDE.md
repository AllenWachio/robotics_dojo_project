# 🌳 Behavior Tree Autonomous Mission Guide

## Overview

The **behavior tree** is a high-level mission controller that orchestrates autonomous navigation tasks. It runs on the **LAPTOP** and commands the robot to execute sequential missions.

---

## 🖥️ Architecture: Where Components Run

### **ON THE RASPBERRY PI:**

```
┌─────────────────────────────────────┐
│  Raspberry Pi (On-board Hardware)   │
├─────────────────────────────────────┤
│  • Arduino Bridge (serial control)  │
│  • LiDAR Driver (USB sensor)        │
│  • Camera Node (Pi camera)          │
│  • Publishes: /odom, /scan, /image  │
└─────────────────────────────────────┘
```

**Why Pi?**

- Direct hardware connections (USB, serial, camera)
- Real-time sensor data capture
- Low-latency motor control
- Must run on the physical robot

---

### **ON THE LAPTOP:**

```
┌─────────────────────────────────────────────────────┐
│  Laptop (Computation & Decision Making)             │
├─────────────────────────────────────────────────────┤
│  • SLAM / AMCL Localization                         │
│  • Nav2 Navigation Stack (path planning, control)   │
│  • RViz2 Visualization                              │
│  • Behavior Tree 🌳 ⭐ (mission orchestration)      │
│  • Sends: Navigation goals, control commands        │
└─────────────────────────────────────────────────────┘
```

**Why Laptop?**

- CPU-intensive computations (SLAM, path planning)
- GUI applications (RViz)
- Easy debugging and development
- High-level decision making
- Wireless communication with robot

---

## 🎯 What the Behavior Tree Does

The behavior tree is a **state machine** that:

1. **Orchestrates missions** - Defines what the robot should do
2. **Sends navigation goals** - Commands Nav2 to move to waypoints
3. **Executes tasks** - Performs actions at each location
4. **Monitors progress** - Tracks success/failure of each step
5. **Sequential execution** - Proceeds step-by-step through mission

### **Current Mission:**

```
RootSequence (executes in order)
├── Move to waypoint 1: (2.1, 0.0)
├── Task 1: Print "hello"
├── Move to waypoint 2: (0, -1.2)
└── Task 2: Print "hi"
```

---

## 🚀 How to Run

### **Step-by-Step Procedure:**

#### **1. Start Hardware on Pi (2 terminals)**

**Pi Terminal 1:**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/pi
./01_arduino_only.sh
```

**Pi Terminal 2:**

```bash
./02_lidar_only.sh
```

**What this does:**

- Starts Arduino bridge (motors, encoders)
- Starts LiDAR sensor
- Publishes `/odom` and `/scan` topics
- Robot hardware is ready

---

#### **2. Start Navigation on Laptop**

Choose ONE navigation option:

**Option A - AMCL Navigation:**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./02_navigation_mode.sh my_map
```

**Option B - SLAM Localization:**

```bash
./02b_slam_localization_mode.sh my_map
```

**Option C - SLAM Navigation (Recommended):**

```bash
./02c_slam_navigation_mode.sh my_map
```

**What this does:**

- Loads the map
- Starts localization (AMCL or SLAM Toolbox)
- Starts Nav2 navigation stack
- Opens RViz for visualization

---

#### **3. Set Initial Pose in RViz**

⚠️ **CRITICAL STEP** - Don't skip this!

1. In RViz, click **"2D Pose Estimate"** button (top toolbar)
2. Click on the map where your robot is located
3. Drag to set the robot's orientation
4. Release mouse button
5. Watch the robot localize (pose converges, particles cluster)

**Verification:**

- Robot model in RViz should align with real robot position
- Pose covariance (ellipse) should be small
- LiDAR scan should align with map walls

---

#### **4. Launch Behavior Tree Mission**

**Laptop Terminal (new terminal):**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./05_behavior_tree_mission.sh
```

**What this does:**

- Runs pre-flight checks (Nav2 running? Localization active?)
- Verifies py_trees is installed
- Launches the behavior tree mission
- Robot starts autonomous execution!

---

## 📊 What You'll See

### **Terminal Output:**

```
╔════════════════════════════════════════════════════════════════╗
║      🤖 Behavior Tree Autonomous Mission Launcher 🌳          ║
╚════════════════════════════════════════════════════════════════╝

📋 Pre-flight Checklist:

🔍 Checking if Nav2 navigation is running...
   ✅ Nav2 navigation stack detected
🔍 Checking localization system...
   ✅ AMCL localization detected
🔍 Checking if Pi hardware nodes are running...
   ✅ Robot hardware detected (odometry publishing)
🔍 Checking if py_trees is installed...
   ✅ py_trees Python package found

🚀 All checks passed! Starting behavior tree mission...

MoveToPoint1: Sending goal to (2.1, 0.0)
MoveToPoint1: Goal accepted, navigating...
MoveToPoint1: Current(0.00, 0.00) -> Target(2.1, 0.0), Distance: 2.10m
MoveToPoint1: Current(0.52, 0.03) -> Target(2.1, 0.0), Distance: 1.59m
...
MoveToPoint1: Navigation completed! Final distance: 0.18m
hello
MoveToPoint2: Sending goal to (0, -1.2)
...
hi
Behavior tree completed successfully!
```

### **In RViz:**

- Green navigation path appears
- Robot model moves along path
- Local costmap updates in real-time
- Goal marker shows target location

---

## 🛠️ Customization

### **Change Waypoints:**

Edit: `~/ros2_ws/src/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py`

Find the `create_root()` function:

```python
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # CHANGE THESE COORDINATES:
    move1 = MoveToPosition("MoveToPoint1", 2.1, 0.0, tolerance=0.2)
    task1 = PrintHello("Task1")
    move2 = MoveToPosition("MoveToPoint2", 0, -1.2, tolerance=0.2)
    task2 = PrintHi("Task2")

    root.add_children([move1, task1, move2, task2])
    return root
```

**Example: 3-waypoint patrol:**

```python
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    move1 = MoveToPosition("Checkpoint1", 1.0, 0.0, tolerance=0.2)
    move2 = MoveToPosition("Checkpoint2", 1.0, 1.0, tolerance=0.2)
    move3 = MoveToPosition("Checkpoint3", 0.0, 1.0, tolerance=0.2)
    move4 = MoveToPosition("ReturnHome", 0.0, 0.0, tolerance=0.2)

    root.add_children([move1, move2, move3, move4])
    return root
```

---

### **Add Custom Tasks:**

Replace `PrintHello` with real actions:

```python
class CaptureImage(py_trees.behaviour.Behaviour):
    """Capture image from camera"""
    def __init__(self, name="CaptureImage"):
        super().__init__(name)
        self.executed = False

    def setup(self, **kwargs):
        node = kwargs.get('node')
        # Create image subscriber
        self.image_sub = node.create_subscription(
            Image, '/image', self.image_callback, 10)
        self.image_received = False

    def image_callback(self, msg):
        # Save image or process it
        print(f"{self.name}: Image captured!")
        self.image_received = True

    def update(self):
        if self.image_received and not self.executed:
            print(f"{self.name}: Processing image...")
            # Add your image processing here
            self.executed = True
            return py_trees.common.Status.SUCCESS
        elif not self.executed:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
```

---

## 🔍 Troubleshooting

### **Error: "Nav2 navigation stack NOT running!"**

**Problem:** Navigation must be running before behavior tree starts.

**Solution:**

```bash
# Start navigation first:
./02c_slam_navigation_mode.sh my_map

# Wait for Nav2 to fully start (~10 seconds)
# Then in another terminal:
./05_behavior_tree_mission.sh
```

---

### **Error: "py_trees NOT installed!"**

**Problem:** Missing Python dependency.

**Solution:**

```bash
pip3 install py_trees py_trees_ros
```

---

### **Robot doesn't move to waypoints**

**Problem:** Initial pose not set or localization poor.

**Solution:**

1. Check in RViz that robot pose is correct
2. Use "2D Pose Estimate" to relocalize
3. Wait for pose covariance to stabilize
4. Verify LiDAR scan aligns with map
5. Then start behavior tree

---

### **Goal rejected or navigation fails**

**Problem:** Waypoint is unreachable (obstacle, outside map, etc.)

**Solution:**

1. Check waypoint coordinates are within the map
2. Verify no obstacles block the path
3. Adjust waypoint coordinates
4. Increase costmap inflation radius if needed

---

### **Behavior tree stops unexpectedly**

**Problem:** One behavior failed, stopping the sequence.

**Solution:**

1. Check terminal output for error messages
2. Verify all ROS topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic echo /amcl_pose
   ros2 topic echo /odom
   ```
3. Restart navigation and behavior tree

---

## 🎓 Understanding the Architecture

### **Layer 1: Behavior Tree (High-level Mission)**

```
"Go to plant 1, inspect, go to plant 2, inspect, return home"
        ↓ (NavigateToPose actions)
```

### **Layer 2: Nav2 (Mid-level Navigation)**

```
"Plan path around obstacles, follow path, avoid collisions"
        ↓ (cmd_vel commands)
```

### **Layer 3: Arduino Bridge (Low-level Control)**

```
"Convert velocity to motor PWM, read encoders, publish odometry"
        ↓ (Serial protocol)
```

### **Layer 4: Hardware (Physical Execution)**

```
"Motors turn, wheels spin, robot moves"
```

---

## 📈 Real-World Applications

### **Agricultural Robot Use Cases:**

**1. Field Inspection:**

```python
# Patrol crop rows, capture images at each plant
for row in range(1, 5):
    move_to_row_start(row)
    for plant in range(1, 10):
        move_to_plant(row, plant)
        capture_and_analyze_image()
        if disease_detected():
            mark_location_for_treatment()
```

**2. Selective Treatment:**

```python
# Navigate to diseased plants and apply treatment
for location in diseased_plant_list:
    navigate_to(location)
    activate_sprayer()
    wait(5_seconds)
    deactivate_sprayer()
return_to_base()
```

**3. Warehouse Automation:**

```python
# Pick up items from shelves
for item in shopping_list:
    navigate_to_shelf(item.location)
    pick_item(item)
    if cart_full():
        navigate_to_dropoff()
        empty_cart()
```

---

## 📚 Additional Resources

**Documentation:**

- Full integration guide: `~/ros2_ws/src/ros_arduino_bridge/PYTREES_INTEGRATION_GUIDE.md`
- Behavior tree README: `~/ros2_ws/src/ros_arduino_bridge/behavior_tree/README.md`
- Py_trees docs: https://py-trees.readthedocs.io/

**Related Scripts:**

- `./QUICK_REFERENCE.sh` - Overall system reference
- `./04_diagnostics.sh` - System health check

---

## ✅ Quick Reference

### **Complete Startup Sequence:**

```bash
# === ON RASPBERRY PI ===
# Terminal 1:
./01_arduino_only.sh

# Terminal 2:
./02_lidar_only.sh

# === ON LAPTOP ===
# Terminal 1:
./02c_slam_navigation_mode.sh my_map
# (Set initial pose in RViz)

# Terminal 2:
./05_behavior_tree_mission.sh
```

### **Mission Monitoring:**

```bash
# Watch behavior tree progress (in terminal output)

# Monitor navigation topics:
ros2 topic echo /navigate_to_pose/_action/status
ros2 topic echo /cmd_vel

# Check robot pose:
ros2 topic echo /amcl_pose
```

---

## 🎯 Summary

| Question               | Answer                                               |
| ---------------------- | ---------------------------------------------------- |
| **Where does it run?** | LAPTOP (not Raspberry Pi)                            |
| **Why laptop?**        | Sends goals to Nav2, monitors AMCL, high-level logic |
| **Prerequisites?**     | Pi hardware + Laptop navigation running              |
| **How to start?**      | `./05_behavior_tree_mission.sh`                      |
| **How to customize?**  | Edit `behavior_tree/robot_navigation_bt.py`          |

---

**Happy autonomous missions! 🤖🌳🚀**
