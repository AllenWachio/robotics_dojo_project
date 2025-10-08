# 🔗 USING YOUR EXISTING FILES

## ✅ All Launch Files Already Exist!

I've updated the competition scripts to use **YOUR EXISTING** launch files instead of creating new ones. Here's what's being used:

---

## 📦 Pi Hardware (Step 1)

**Script**: `01_pi_hardware.sh`

**Calls YOUR existing scripts**:

1. **Arduino**: `deployment/scripts/pi/01_arduino_only.sh`

   - Which launches: `arduino_only.launch.py`
   - ✅ Arduino bridge (motors, encoders, IMU, servos)
   - ✅ Robot State Publisher (TF tree)
   - ✅ EKF sensor fusion (fuses encoders + IMU)
   - ✅ Publishes `/odom`, `/imu/data`, `/odometry/filtered`

2. **LiDAR**: `deployment/scripts/pi/02_lidar_only.sh`

   - Which launches: `lidar.launch.py`
   - ✅ SLLIDAR A1 driver
   - ✅ Publishes `/scan`

3. **Camera**: `rpi_camera_package/scripts/pi/run_camera.sh`
   - Which launches: `camera_compressed.launch.py`
   - ✅ V4L2 camera capture
   - ✅ Image compression (JPEG quality 80)
   - ✅ Publishes `/camera/image_raw/compressed`

---

## 💻 Laptop Processing (Step 2)

**Script**: `02_laptop_processing.sh`

**Calls YOUR existing scripts**:

1. **Navigation**: `deployment/scripts/laptop/02c_slam_navigation_mode.sh`

   - Which launches: `laptop_navigation_slam.launch.py`
   - ✅ Map Server (loads gamefield map)
   - ✅ SLAM Toolbox Localization
   - ✅ Nav2 Complete Stack:
     - Controller Server
     - Planner Server
     - Behavior Server
     - BT Navigator
     - Waypoint Follower
     - Velocity Smoother
   - ✅ Lifecycle Manager
   - ✅ RViz2 with navigation interface

2. **Vision Processing**: `rpi_camera_package/scripts/laptop/run_full_processing.sh`
   - Which launches: `full_processing.launch.py`
   - ✅ Color Detection Node (OpenCV HSV)
   - ✅ Disease Detection Node (ML model)
   - ✅ Both subscribe to `/camera/image_raw/compressed`
   - ✅ Comprehensive pre-flight checks (PyTorch, OpenCV, PIL)

---

## 🌳 Behavior Tree (Step 3)

**Script**: `03_run_mission.sh`

**Runs NEW file**: `competition_mission.py`

- ✅ Uses py_trees behavior tree
- ✅ Subscribes to YOUR existing topics:
  - `/amcl_pose` (from laptop_navigation.launch.py)
  - `/scan` (from pi_robot_hardware.launch.py)
  - `/odom` (from pi_robot_hardware.launch.py)
  - `/camera/color_detection` (from full_processing.launch.py)
  - `/inference_result` (from full_processing.launch.py)
  - `/color_sensor/rgb` (from pi_robot_hardware.launch.py)
- ✅ Publishes to YOUR existing topics:
  - `/cmd_vel` (to arduino bridge)
  - `/camera_servo/command` (to arduino)
  - `/tipper_servo/command` (to arduino)
  - `/conveyor/command` (to arduino)

---

## 🗺️ Topic Flow Map

```
RASPBERRY PI                           LAPTOP
───────────                           ──────

pi_robot_hardware.launch.py          laptop_navigation.launch.py
  ├─ Arduino Bridge                     ├─ Map Server
  │   ├─ publishes /odom         ─────→ ├─ AMCL (subscribes to /odom)
  │   ├─ publishes /imu/data            ├─ Nav2 Stack
  │   ├─ publishes /color_sensor/rgb    │   ├─ Planner
  │   └─ subscribes /cmd_vel ←──────────┤   └─ Controller (publishes /cmd_vel)
  │                                     │
  └─ LiDAR                              └─ RViz2
      └─ publishes /scan        ─────→ (costmap subscribes to /scan)

camera_compressed.launch.py           full_processing.launch.py
  └─ V4L2 Camera                        ├─ Color Detection Node
      └─ publishes                      │   ├─ subscribes /camera/image_raw/compressed
         /camera/image_raw/compressed ──┤   └─ publishes /camera/color_detection
                                        │
                                        └─ Disease Detection Node
                                            ├─ subscribes /camera/image_raw/compressed
                                            └─ publishes /inference_result

                                      competition_mission.py (Behavior Tree)
                                        ├─ Reads all published topics
                                        ├─ Makes autonomous decisions
                                        └─ Sends commands to Pi
```

---

## 📂 File Structure Summary

```
ros_arduino_bridge/
├── deployment/
│   ├── pi/launch/
│   │   └── pi_robot_hardware.launch.py     ← USING THIS (Pi hardware)
│   └── laptop/launch/
│       └── laptop_navigation.launch.py     ← USING THIS (Nav2)
│
└── behavior_tree/
    └── competition_2025/
        ├── launch/
        │   ├── 01_pi_hardware.sh           ← Calls YOUR Pi launch files
        │   ├── 02_laptop_processing.sh     ← Calls YOUR laptop launch files
        │   └── 03_run_mission.sh           ← Runs NEW behavior tree
        │
        ├── competition_mission.py          ← NEW (uses YOUR topics)
        └── behaviors/                       ← NEW (behavior nodes)

rpi_camera_package/
├── launch/
│   ├── pi/
│   │   └── camera_compressed.launch.py     ← USING THIS (Pi camera)
│   └── laptop/
│       └── full_processing.launch.py       ← USING THIS (vision processing)
```

---

## 🎯 What's NEW vs What's REUSED

### ✅ REUSED (Your Existing Files):

1. **pi_robot_hardware.launch.py** - Arduino + LiDAR
2. **camera_compressed.launch.py** - Pi camera with compression
3. **laptop_navigation.launch.py** - Complete Nav2 stack
4. **full_processing.launch.py** - Color + disease detection

### 🆕 NEW (Competition-Specific):

1. **competition_mission.py** - Main behavior tree mission
2. **behaviors/** - Modular behavior node library
3. **01_pi_hardware.sh** - Convenience script (calls YOUR launch files)
4. **02_laptop_processing.sh** - Convenience script (calls YOUR launch files)
5. **03_run_mission.sh** - Mission launcher script

---

## 🚀 Why This Works

**Your existing infrastructure** provides:

- ✅ Hardware interfaces (Arduino, LiDAR, Camera)
- ✅ Navigation stack (Nav2 fully configured)
- ✅ Vision processing (Color + Disease detection)

**New behavior tree** adds:

- ✅ Autonomous decision-making
- ✅ Mission sequencing (phases 1-4)
- ✅ Sensor data integration
- ✅ Competition-specific logic

**Together** they create:

- ✅ Fully autonomous competition robot
- ✅ Split Pi/Laptop architecture maintained
- ✅ No duplicate code
- ✅ Uses your tested, working launch files

---

## 📝 No Files Deleted or Modified

**Your original files are UNTOUCHED:**

- ✅ All deployment launch files: unchanged
- ✅ All camera launch files: unchanged
- ✅ All config files: unchanged
- ✅ All existing Python nodes: unchanged

**Only NEW files added:**

- `competition_2025/` folder with behavior tree code
- Shell scripts that call your existing launch files

---

## 🎓 How It Integrates

### Example: Navigation

```python
# In competition_mission.py
from behaviors import MoveToPosition

# This behavior:
move = MoveToPosition("GoToGoal", 1.5, 0.8)

# Uses YOUR existing infrastructure:
# 1. Subscribes to /amcl_pose (from laptop_navigation.launch.py)
# 2. Sends goal to Nav2 (from laptop_navigation.launch.py)
# 3. Nav2 publishes /cmd_vel
# 4. Arduino (from pi_robot_hardware.launch.py) receives /cmd_vel
# 5. Robot moves!

# NO NEW NAVIGATION CODE - just orchestrates YOUR system!
```

### Example: Color Detection

```python
# In competition_mission.py
from behaviors import ReadColorSensor, MonitorCameraForColor

# These behaviors:
read_color = ReadColorSensor("ReadCargo")
monitor_camera = MonitorCameraForColor("MonitorZone", "red")

# Use YOUR existing nodes:
# - ReadColorSensor subscribes to /color_sensor/rgb
#   (published by arduino in pi_robot_hardware.launch.py)
# - MonitorCameraForColor subscribes to /camera/color_detection
#   (published by color_detection_node in full_processing.launch.py)

# NO NEW VISION CODE - just reads YOUR topics!
```

---

## ✨ Benefits

1. **Reuses your tested code** - No reinventing the wheel
2. **Maintains split architecture** - Pi/Laptop separation preserved
3. **Easy to debug** - Can test components independently
4. **No conflicts** - New code in separate folder
5. **Simple to launch** - 3 shell scripts orchestrate everything

---

## 🏁 Competition Day Workflow

```bash
# Step 1: On Pi (calls YOUR existing hardware scripts)
./01_pi_hardware.sh
# → deployment/scripts/pi/01_arduino_only.sh (Arduino + EKF)
# → deployment/scripts/pi/02_lidar_only.sh (LiDAR)
# → rpi_camera_package/scripts/pi/run_camera.sh (Camera)

# Step 2: On Laptop (calls YOUR existing processing scripts)
./02_laptop_processing.sh
# → deployment/scripts/laptop/02c_slam_navigation_mode.sh (Nav2)
# → rpi_camera_package/scripts/laptop/run_full_processing.sh (Vision)

# Step 3: On Laptop (runs NEW behavior tree)
./03_run_mission.sh
# → python3 competition_mission.py
```

**Result**: Your existing infrastructure + new autonomous behavior tree = Competition-ready robot! 🏆
