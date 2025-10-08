# Competition 2025 Code - Organized Structure

## 📁 Folder Organization

```
competition_2025/
├── behaviors/              # All behavior node modules
│   ├── __init__.py        # Easy imports
│   ├── navigation.py      # MoveToPosition, ReverseDistance, etc.
│   ├── color_sensor.py    # RGB sensor behaviors
│   ├── camera.py          # Pi camera behaviors
│   ├── disease_detection.py  # ML model integration
│   └── actuators.py       # Servo/motor control
│
├── competition_mission.py  # MAIN MISSION FILE (run this!)
│
├── launch/
│   └── (launch files will go here)
│
├── testing/
│   └── (test scripts will go here)
│
└── docs/
    └── README.md (this file)
```

## 🚀 Quick Start

```bash
# Terminal 1: Launch navigation
cd ~/ros2_ws
ros2 launch ros_arduino_bridge laptop_navigation.launch.py

# Terminal 2: Run competition mission
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025
python3 competition_mission.py
```

## 🧭 How Movement Works

### Direction System

The robot uses **MAP FRAME coordinates** provided by AMCL:

```
    ↑ Y-axis (North)
    |
    |
    └────→ X-axis (East)
```

### Navigation Flow

1. **AMCL Localization**: Tracks robot position (x, y, yaw) on map
2. **Nav2 Planning**: Calculates optimal path considering obstacles
3. **DWB Controller**: Executes path with dynamic obstacle avoidance
4. **LiDAR Costmap**: Provides real-time obstacle detection

### Example Movement

```python
MoveToPosition("GoToGoal", 2.0, 1.5, tolerance=0.2)
```

This means:

- Navigate to map coordinates (2.0, 1.5)
- X = 2.0m East from origin
- Y = 1.5m North from origin
- Stop when within 0.2m of goal

The **path taken** depends on:

- Current robot position (from AMCL `/amcl_pose`)
- Obstacles in costmap (from LiDAR `/scan`)
- Nav2 planner algorithm (Dijkstra/A\*)

## 📍 Waypoints

Defined in `competition_mission.py`:

| Waypoint        | Coordinates | Description             |
| --------------- | ----------- | ----------------------- |
| start           | (0.0, 0.0)  | Robot starting position |
| disease_station | (0.5, 1.5)  | Plant display area      |
| loading_bay     | (0.3, 0.8)  | Cargo pickup zone       |
| maze_entrance   | (0.8, 0.0)  | Path transition         |
| green_delivery  | (1.5, -1.5) | Green zone              |
| red_delivery    | (1.5, -0.8) | Red zone                |
| blue_delivery   | (1.5, -0.2) | Blue zone               |

⚠️ **VERIFY THESE** using RViz before competition!

## 🎯 Mission Phases

### 1. Disease Detection (Optional)

- Navigate to plant display
- Run ML inference
- Log result
- Return to start

### 2. Cargo Loading

- Navigate to loading bay
- Reverse into bay
- Read RGB sensor (red/blue/green/yellow)
- Exit bay

### 3. Maze Navigation

- Navigate to delivery zone based on cargo color
- Monitor camera for zone confirmation
- Dynamic obstacle avoidance via LiDAR

### 4. Cargo Delivery

- Verify correct zone
- Reverse into delivery bay
- Offload cargo:
  - Try conveyor belt
  - If fails, add tipper servo tilt
- Reset robot position

## 🔧 Behavior Nodes

### Navigation (`behaviors/navigation.py`)

- **MoveToPosition**: Nav2 autonomous navigation
- **ReverseDistance**: Open-loop backwards movement
- **Turn180Degrees**: Rotate in place
- **StopRobot**: Emergency stop

### Sensors (`behaviors/color_sensor.py`, `camera.py`)

- **ReadColorSensor**: RGB sensor reading
- **MonitorCameraForColor**: Camera color detection
- **VerifyColorMatch**: Confirm delivery zone

### Disease Detection (`behaviors/disease_detection.py`)

- **WaitForDiseaseDetection**: ML inference
- **LogDiseaseResult**: Format output
- **CheckDiseaseDetectionRequired**: Enable/disable phase

### Actuators (`behaviors/actuators.py`)

- **ActivateCameraServo**: Pan camera
- **ActivateTipperServo**: Tilt robot
- **ActivateConveyorBelt**: Offload cargo
- **ResetTipperServo**: Return to level

## 📖 Inspired By

This implementation is based on the working reference code you provided, which demonstrated:

- ✅ Shared AMCL pose subscription (singleton pattern)
- ✅ Nav2 action client with async callbacks
- ✅ Goal tolerance configuration
- ✅ Real-time distance monitoring
- ✅ Proper state machine implementation

## 🆚 Differences from Old Code

**Old Location**: `behavior_tree/nodes/` (mixed with original files)
**New Location**: `behavior_tree/competition_2025/` (clean, separate)

**Key Improvements**:

1. **Organized structure**: Separate folder for competition code
2. **Clear navigation**: Detailed comments about movement direction
3. **Proven approach**: Based on your working reference implementation
4. **Easy imports**: Single `from behaviors import ...` statement
5. **Well documented**: Every behavior explains HOW movement works

## 🧪 Testing

```bash
# Test individual behaviors
cd testing/
python3 test_navigation.py
python3 test_sensors.py

# Test waypoints
cd ..
python3 test_waypoints.py list
python3 test_waypoints.py goto loading_bay
```

## 📝 Next Steps

1. **Verify waypoints** using RViz or test_waypoints.py
2. **Calibrate sensors** (color thresholds, camera HSV)
3. **Test mission phases** individually
4. **Run full mission** on practice field
5. **Time optimization** (target: 2.5-3 minutes)

## 🏆 Competition Day

See `CHEAT_SHEET.md` in parent directory for 5-terminal setup procedure.

---

**Questions?** Check the detailed comments in each behavior file!
