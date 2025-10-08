# 🏆 SPLIT ARCHITECTURE DEPLOYMENT GUIDE

## Architecture Overview

Your system uses **split processing** between Raspberry Pi and Laptop:

```
┌─────────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI (Robot)                         │
│                 Low-compute hardware tasks                      │
├─────────────────────────────────────────────────────────────────┤
│  ✓ Arduino Bridge (motors, encoders, IMU, RGB sensor, servos)  │
│  ✓ LiDAR sensor (raw scan data)                                │
│  ✓ Pi Camera (image capture & compression)                     │
│  ✓ Robot State Publisher (TF transforms)                       │
│                                                                 │
│  Publishes topics → Laptop subscribes automatically            │
└─────────────────────────────────────────────────────────────────┘
                            ↓ (ROS2 network)
┌─────────────────────────────────────────────────────────────────┐
│                      LAPTOP (Base Station)                      │
│                  Heavy compute processing                       │
├─────────────────────────────────────────────────────────────────┤
│  ✓ Nav2 Navigation Stack (path planning, AMCL, costmaps)      │
│  ✓ Disease Detection ML Model (TensorFlow/PyTorch)            │
│  ✓ Camera Color Detection (OpenCV HSV processing)             │
│  ✓ Behavior Tree (py_trees mission execution)                 │
│  ✓ RViz (visualization & monitoring)                           │
│                                                                 │
│  Subscribes to Pi topics, publishes /cmd_vel commands          │
└─────────────────────────────────────────────────────────────────┘
```

## Why This Architecture?

**Benefits:**

- ✅ **Pi stays lightweight**: No heavy Nav2/ML processing
- ✅ **Laptop has full compute**: Can run TensorFlow, Nav2, RViz
- ✅ **Modular**: Can test components independently
- ✅ **Network transparent**: ROS2 DDS handles discovery
- ✅ **Development friendly**: Code on laptop, test on robot

## 🚀 How to Launch (3-Step Process)

### Step 1: Start Pi Hardware (on Raspberry Pi)

```bash
# SSH into Raspberry Pi
ssh pi@<robot_ip>

# Navigate to launch folder
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025/launch

# Run Pi hardware script
./01_pi_hardware.sh
```

**What this does:**

- Launches Arduino bridge → publishes `/odom`, `/imu/data`, `/color_sensor/rgb`
- Launches LiDAR → publishes `/scan`
- Launches camera → publishes `/camera/image_raw/compressed`
- Publishes TF tree (`base_link`, `laser`, `camera_link`)

**Leave this running!** Do NOT close this terminal.

---

### Step 2: Start Laptop Processing (on Laptop)

```bash
# On your laptop
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025/launch

# Run laptop processing script
./02_laptop_processing.sh
```

**What this does:**

- Loads competition map (`gamefield.yaml`)
- Starts Nav2 navigation stack
- Starts AMCL localization → publishes `/amcl_pose`
- Starts disease detection ML model → subscribes to `/camera/image_raw/compressed`
- Starts color detection → subscribes to `/camera/image_raw/compressed`
- Opens RViz for visualization

**Set Initial Pose:**

1. In RViz, click "2D Pose Estimate" button
2. Click on map where robot actually is
3. Drag to set orientation
4. Watch particles converge around robot

**Leave this running!** Do NOT close this terminal.

---

### Step 3: Run Behavior Tree Mission (on Laptop)

Open a **new terminal** on laptop:

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025/launch

# Run mission script
./03_run_mission.sh
```

**What this does:**

- Checks all prerequisites (Nav2, AMCL, Pi hardware)
- Asks you to confirm initial pose is set
- Launches `competition_mission.py` behavior tree
- **Robot becomes fully autonomous!**

---

## 🤖 Autonomy Explained

### "Once I start, will it be fully autonomous?"

**YES!** Once you run `03_run_mission.sh`, the robot is **completely autonomous**:

1. **You do nothing on the Pi** - Hardware runs continuously
2. **You do nothing on laptop** - Behavior tree controls everything
3. **Robot executes mission:**
   - Navigates autonomously (Nav2 handles paths)
   - Detects diseases (ML model runs automatically)
   - Reads cargo color (RGB sensor automatic)
   - Delivers to correct zone (camera monitors color)
   - Offloads cargo (actuators activate automatically)

**You only intervene if:**

- Emergency stop (Ctrl+C)
- Mission fails (behavior tree will report)
- Manual debugging (you can teleop if needed)

### How the Behavior Tree Stays Autonomous

```python
# In competition_mission.py

def update(self):
    """
    MoveToPosition behavior automatically:
    1. Reads current pose from /amcl_pose
    2. Sends goal to Nav2 action server
    3. Monitors progress
    4. Returns SUCCESS when arrived

    YOU DON'T NEED TO DO ANYTHING!
    """
    if not self.goal_sent:
        # Send goal to Nav2
        goal_msg = NavigateToPose.Goal()
        self.action_client.send_goal_async(goal_msg)
        # Nav2 handles: path planning, obstacle avoidance, execution

    # Check if arrived
    if distance < self.tolerance:
        return SUCCESS

    return RUNNING  # Keep waiting
```

**Key point**: Behavior tree nodes are **reactive**:

- They send commands once
- Then monitor topics for updates
- Return status based on sensor data
- No human input required!

---

## 📡 Topic Flow Diagram

```
RASPBERRY PI                      LAPTOP
-----------                      --------

Arduino Bridge                   Behavior Tree
    ↓                                ↓
  /odom ──────────────────────→ AMCL (localization)
  /imu/data                         ↓
  /color_sensor/rgb ────────→ ReadColorSensor behavior
    ↑                                ↓
    └─────────────────────── /cmd_vel ← Nav2 Controller

LiDAR                            Nav2 Stack
    ↓                                ↓
  /scan ──────────────────────→ Costmap (obstacle avoidance)
                                     ↓
Pi Camera                        Path Planner
    ↓                                ↓
  /camera/image_raw/compressed   Optimal path
    ├──────────────────────────→ Disease Detection ML
    └──────────────────────────→ Color Detection OpenCV
                                     ↓
                                 VerifyColorMatch behavior
```

**Autonomous Loop:**

1. Pi publishes sensor data
2. Laptop Nav2 processes → sends `/cmd_vel`
3. Pi Arduino executes `/cmd_vel` → robot moves
4. Pi publishes new sensor data
5. Loop continues until mission complete!

---

## 🔧 Troubleshooting Split Architecture

### Problem: Laptop can't see Pi topics

```bash
# On laptop, check if Pi topics are visible
ros2 topic list | grep odom

# If no output, check network
ping <pi_ip>

# Check ROS_DOMAIN_ID matches
# On Pi:
echo $ROS_DOMAIN_ID

# On laptop:
echo $ROS_DOMAIN_ID

# Should be same number (usually 0)
```

**Fix**: Set same domain ID:

```bash
# In ~/.bashrc on BOTH Pi and laptop
export ROS_DOMAIN_ID=0
```

### Problem: Behavior tree can't send commands to robot

Check if laptop can publish to Pi:

```bash
# On laptop, try publishing test velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once

# Robot should move slightly
```

### Problem: ML model slows down Pi

**This is why we use split architecture!**

ML runs on **laptop only**:

- Pi just sends compressed images
- Laptop runs TensorFlow model
- Pi never touches ML code

### Problem: RViz shows "No transform from map to base_link"

**Cause**: AMCL not localized yet

**Fix**:

1. Set initial pose in RViz (2D Pose Estimate)
2. Drive robot slightly (teleop) so AMCL converges
3. Watch particles in RViz collapse around robot

---

## 📊 Resource Usage

### Raspberry Pi:

- **CPU**: ~30-40% (Arduino + LiDAR + Camera)
- **RAM**: ~200-300 MB
- **Network**: ~5-10 Mbps (compressed images)
- **Temperature**: Should stay < 70°C

### Laptop:

- **CPU**: ~60-80% (Nav2 + ML + RViz)
- **RAM**: ~2-3 GB
- **GPU**: Optional (speeds up ML inference)
- **Network**: ~5-10 Mbps (receiving sensor data)

---

## 🎯 Competition Day Checklist

### On Raspberry Pi:

- [ ] SSH connection working
- [ ] Arduino connected (`ls /dev/serial/by-id/`)
- [ ] LiDAR connected
- [ ] Camera working (`v4l2-ctl --list-devices`)
- [ ] `01_pi_hardware.sh` running
- [ ] Terminal shows "Publishing odometry"

### On Laptop:

- [ ] Can see Pi topics (`ros2 topic list | grep odom`)
- [ ] Map file exists (`~/ros2_ws/maps/gamefield.yaml`)
- [ ] `02_laptop_processing.sh` running
- [ ] RViz shows map and robot
- [ ] Initial pose set in RViz
- [ ] Nav2 ready (`ros2 node list | grep amcl`)

### Start Mission:

- [ ] Run `./03_run_mission.sh`
- [ ] Watch behavior tree output
- [ ] Monitor RViz for navigation
- [ ] DO NOT TOUCH ANYTHING!
- [ ] Let robot complete mission autonomously

---

## 🏁 Summary

**Your split architecture maintains full autonomy:**

1. **Pi handles hardware** (continuously running)
2. **Laptop handles processing** (Nav2, ML, behavior tree)
3. **Behavior tree controls robot** (no human input needed)
4. **ROS2 network connects everything** (transparent communication)

**You start 3 scripts, then watch the robot work!**

The robot is **truly autonomous** because:

- Behavior tree makes all decisions
- Nav2 handles all navigation
- Sensors provide feedback loops
- No manual intervention required

**Competition flow:**

```
Human: ./01_pi_hardware.sh  (on Pi)
Human: ./02_laptop_processing.sh  (on laptop)
Human: Set initial pose in RViz
Human: ./03_run_mission.sh
Human: 👀 Watch robot complete mission autonomously! 🎉
```
