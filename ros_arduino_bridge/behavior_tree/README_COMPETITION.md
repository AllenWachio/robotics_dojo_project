# Competition Mission - Quick Start Guide

## 🎯 Mission Overview

Autonomous robot competition with 4 main phases:

1. **Disease Detection** - Navigate to plant, classify disease
2. **Cargo Loading** - Navigate to loading bay, identify cargo color
3. **Maze Navigation** - Navigate through maze avoiding obstacles
4. **Cargo Delivery** - Reverse into delivery bay, offload cargo

**Target Time**: 2.5-3 minutes

---

## 📁 Project Structure

```
behavior_tree/
├── nodes/                          # Modular behavior nodes
│   ├── __init__.py                # Easy imports
│   ├── color_sensor_behaviors.py  # Color sensor reading
│   ├── camera_behaviors.py        # Pi camera detection
│   ├── disease_detection_behaviors.py
│   ├── navigation_behaviors.py    # Movement & Nav2
│   └── motor_control_behaviors.py # Servos & conveyor
│
├── competition_mission.py         # MAIN MISSION FILE
├── launch_competition.py          # Easy launcher
├── test_behaviors.py              # Individual testing
└── README_COMPETITION.md          # This file
```

---

## 🚀 Quick Start

### Pre-Competition Setup

1. **Update Waypoints** in `competition_mission.py`:

   ```python
   WAYPOINTS = {
       'start': (0.0, 0.0),
       'disease_station': (1.0, 0.5),  # ← UPDATE!
       'loading_bay': (2.0, 0.0),      # ← UPDATE!
       'red_delivery': (4.5, 1.0),     # ← UPDATE!
       'blue_delivery': (4.5, -1.0),   # ← UPDATE!
   }
   ```

2. **Test Individual Components**:

   ```bash
   cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree

   # Test color sensor
   python3 test_behaviors.py color_sensor

   # Test camera
   python3 test_behaviors.py camera

   # Test motors (servos, conveyor)
   python3 test_behaviors.py motors
   ```

3. **Calibrate Sensors**:
   - Color sensor: Test with actual competition cubes
   - Camera: Adjust lighting and color thresholds
   - Conveyor: Test offload timing

---

## 🏁 Competition Day Procedure

### Step 1: Start Robot Systems

**Terminal 1** (on Robot/Pi):

```bash
ros2 launch ros_arduino_bridge full_slam_test.launch.py
```

**Terminal 2** (on Laptop):

```bash
ros2 launch ros_arduino_bridge laptop_navigation.launch.py
```

**Terminal 3** (on Laptop):

```bash
ros2 run rpi_camera_package color_detection_node
```

**Terminal 4** (on Laptop):

```bash
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node
```

### Step 2: Verify Everything Running

```bash
# Check all nodes active
ros2 node list

# Should see:
# /ros_arduino_bridge
# /color_detection_node
# /disease_detection_node
# /amcl
# /controller_server
# /planner_server
# etc.
```

### Step 3: Launch Mission

**Terminal 5** (on Laptop):

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree

# Dry run (check tree structure)
python3 launch_competition.py --dry-run

# Full mission
python3 launch_competition.py

# Skip disease detection (if not required)
python3 launch_competition.py --no-disease
```

### Step 4: Monitor Progress

Watch for log messages:

- `🔬 DISEASE DETECTION RESULT`
- `💡 LED ON, reading color sensor...`
- `✓ Detected color: RED`
- `📷 Monitoring camera for 'red' color...`
- `✓ Camera detected color! Stopping navigation.`
- `🚛 Conveyor running...`

---

## 🧪 Testing Strategy

### Phase 1: Component Testing

Test each subsystem independently:

```bash
# 1. Color sensor
python3 test_behaviors.py color_sensor
# Place colored cube in sensor, verify detection

# 2. Camera
python3 test_behaviors.py camera
# Show colored object to camera, verify detection

# 3. Disease detection
python3 test_behaviors.py disease
# Show plant leaf to camera, verify classification

# 4. Motors
python3 test_behaviors.py motors
# Verify servos move, conveyor activates
```

### Phase 2: Navigation Testing

Test movement in controlled area:

```bash
# ⚠️ ROBOT WILL MOVE!
python3 test_behaviors.py navigation
```

### Phase 3: Integration Testing

Test full mission with mock sensors:

```bash
# TODO: Create mock sensor publishers for testing
ros2 run ... mock_color_sensor
ros2 run ... mock_camera
```

### Phase 4: Full Mission Rehearsal

Run complete mission on actual competition field.

---

## 🔧 Configuration

### Sensor Thresholds

Edit in behavior node files:

**Color Sensor** (`nodes/color_sensor_behaviors.py`):

```python
# Line ~90
if r > 0.5 and r > g * 1.5 and r > b * 1.5:
    detected_color = 'red'
```

**Camera** (`rpi_camera_package/laptop_nodes/color_detection_node.py`):

```python
self.color_ranges = {
    'red': ([0, 120, 70], [10, 255, 255]),
    'blue': ([110, 50, 50], [130, 255, 255]),
}
```

### Timing Parameters

**Conveyor Duration** (`competition_mission.py`):

```python
# Line ~470
conveyor1 = ActivateConveyorBelt("ConveyorAttempt1", duration=4.0, speed=200)
```

**Movement Speeds** (`nodes/navigation_behaviors.py`):

```python
# Reverse speed
ReverseDistance("Reverse", distance_meters=0.5, linear_speed=0.15)

# Nav2 speeds in nav2_params.yaml
max_vel_x: 0.5
max_vel_theta: 2.0
```

---

## 🐛 Troubleshooting

### Mission Won't Start

**Check prerequisites**:

```bash
python3 launch_competition.py --dry-run
```

If fails, manually verify:

```bash
ros2 node list | grep ros_arduino_bridge
ros2 topic list | grep /color_sensor
ros2 action list | grep navigate_to_pose
```

### Color Sensor Not Working

**Check topics**:

```bash
ros2 topic echo /color_sensor/rgb
# Should see RGB values when cube placed
```

**Manual test**:

```bash
ros2 topic pub /color_sensor/led std_msgs/Bool "data: true"
# LED should turn on
```

### Camera Not Detecting

**Check image stream**:

```bash
ros2 topic hz /camera/image_raw/compressed
# Should see ~30 Hz
```

**Check detection output**:

```bash
ros2 topic echo /camera/color_detection
# Should see color names when shown to camera
```

### Navigation Fails

**Check AMCL localization**:

```bash
ros2 topic echo /amcl_pose
# Should see valid x, y coordinates
```

**Check map**:

```bash
ros2 run nav2_map_server map_saver_cli -f test_map
# Verify map file exists and is correct
```

### Conveyor Not Activating

**Check Arduino connection**:

```bash
ros2 topic echo /stepper/debug
# Should see Arduino responses
```

**Manual test**:

```bash
ros2 topic pub /stepper/command std_msgs/String "data: 'conveyor:200:5000'"
```

---

## ⚡ Performance Optimization

### Reduce Mission Time

1. **Increase navigation speed** (if safe):

   ```yaml
   # In nav2_params.yaml
   max_vel_x: 0.6 # Was 0.5
   ```

2. **Reduce stabilization delays**:

   ```python
   # In competition_mission.py
   StopRobot("Stabilize", duration=1.0)  # Was 2.0
   ```

3. **Faster conveyor**:

   ```python
   ActivateConveyorBelt("Conveyor", duration=3.0, speed=255)  # Max speed
   ```

4. **Parallel operations**:
   ```python
   # Run conveyor while reversing (if safe)
   py_trees.composites.Parallel(...)
   ```

---

## 🎯 Competition Checklist

### Pre-Competition (1 hour before)

- [ ] Charge battery fully
- [ ] Verify all hardware connections
- [ ] Test LiDAR (spin and data output)
- [ ] Test color sensor with competition cubes
- [ ] Test camera with competition colors
- [ ] Verify conveyor belt operation
- [ ] Upload latest code to robot
- [ ] Test emergency stop

### Setup at Venue (30 min before)

- [ ] Place robot at starting position
- [ ] Verify WiFi connection
- [ ] Launch all ROS2 nodes
- [ ] Verify `/ros2 node list` shows all nodes
- [ ] Check AMCL localization on RViz
- [ ] Verify map loaded correctly
- [ ] Test manual teleoperation
- [ ] Load map coordinates for venue

### Just Before Run

- [ ] Clear competition field of obstacles
- [ ] Reset robot to start position
- [ ] Verify battery level > 80%
- [ ] All systems green (`launch_competition.py` checks pass)
- [ ] Start timer on launch

### During Run

- Monitor terminal output for errors
- **DO NOT INTERVENE** unless critical failure
- Log any issues for post-run analysis
- Take video of run

### After Run

- [ ] Note final time
- [ ] Download logs
- [ ] Review behavior tree execution
- [ ] Identify optimization opportunities

---

## 📞 Emergency Commands

### Stop Robot Immediately

```bash
# Publish zero velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0}, angular: {z: 0}}"
```

### Cancel Navigation

```bash
# Cancel current goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose --cancel
```

### Reset Localization

```bash
# Republish initial pose
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "..."
```

---

## 📊 Expected Performance

| Phase             | Duration    | Critical Sensors    |
| ----------------- | ----------- | ------------------- |
| Disease Detection | 20-30s      | Camera, Servo       |
| Cargo Loading     | 15-20s      | Color Sensor        |
| Maze Navigation   | 40-60s      | LiDAR, Camera, Nav2 |
| Cargo Delivery    | 20-30s      | Conveyor, Tipper    |
| **Total**         | **95-140s** | **1.5-2.3 min**     |

---

## 🎓 Understanding the Code

### Behavior Tree Structure

```
CompetitionMission (Sequence)
├── DiseaseDetectionPhase (Selector - can skip)
│   ├── Check if enabled
│   └── Disease sequence
│
├── CargoLoadingPhase (Sequence)
│   ├── Navigate to loading
│   ├── Reverse into bay
│   ├── Read color (with timeout fallback)
│   └── Exit bay
│
├── MazeNavigationPhase (Sequence)
│   ├── Enter maze
│   └── Navigate to delivery (color-based selector)
│       ├── Red branch
│       ├── Blue branch
│       └── Manual fallback
│
└── CargoDeliveryPhase (Sequence)
    ├── Verify color match
    ├── Turn 180°
    ├── Reverse into delivery
    ├── Offload (with retry)
    │   ├── Attempt 1: Conveyor only
    │   ├── Check if cleared
    │   └── Attempt 2: Conveyor + Tipper
    └── Exit bay
```

### Key Design Patterns

1. **Selector for Fallbacks**: Try primary, fallback to secondary
2. **Sequence for Steps**: Do A, then B, then C
3. **Blackboard for Data**: Share sensor data between behaviors
4. **Timeout for Safety**: Never block forever - allow manual intervention

---

## 📝 Code Modification Examples

### Add New Color

1. Add to `WAYPOINTS`:

   ```python
   'yellow_delivery': (4.0, 2.0),
   ```

2. Add to delivery selector:
   ```python
   yellow_branch = py_trees.composites.Sequence("YellowDeliveryBranch", memory=True)
   yellow_check = CheckBlackboardColor("CheckYellow", expected='yellow')
   yellow_nav = MoveToPositionWithCameraMonitor("NavigateToYellowZone", *WAYPOINTS['yellow_delivery'])
   yellow_branch.add_children([yellow_check, yellow_nav])
   selector.add_children([..., yellow_branch, ...])
   ```

### Change Offload Sequence

Replace `create_offload_with_retry()` in `competition_mission.py`:

```python
def create_offload_with_retry():
    # Your custom offload logic
    root = py_trees.composites.Sequence("CustomOffload", memory=True)
    # Add your behaviors
    return root
```

### Add Pre-Navigation Check

Insert before maze navigation:

```python
# In create_competition_tree()
battery_check = CheckBatteryLevel("BatteryCheck", min_voltage=11.0)
root.add_children([
    ...,
    battery_check,  # ← New check
    create_maze_navigation_phase(),
    ...
])
```

---

## 🏆 Good Luck!

Remember:

- **Test everything** before competition
- **Stay calm** during the run
- **Document issues** for improvement
- **Have fun!** 🎉

---

**Author**: Robotics Dojo 2025  
**Last Updated**: October 7, 2025
