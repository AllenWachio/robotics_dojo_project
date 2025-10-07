# Cube Pickup and Delivery Mission - Complete Implementation Guide

## 🎯 Mission Overview

This behavior tree implements a complete autonomous cube pickup and delivery workflow:

1. **Navigate to Pickup (Point 1: 2.1, 0.0)**
2. **Stop and identify cube color** (red or blue) using TCS34725 color sensor
3. **Store color data** in behavior tree blackboard
4. **Navigate to Delivery (Point 2)** while Pi camera actively searches for matching color
5. **Stop when camera detects matching color**
6. **Wait for offload confirmation** (color sensor clears - cube removed)
7. **Execute 180° turn**
8. **Reverse 4cm**
9. **Activate stepper motor** for final offload mechanism

---

## 📁 Files Created

### 1. **sensor_behaviors.py**
Contains all sensor and actuator behavior implementations:
- `ReadColorSensor` - Reads TCS34725 RGB sensor, classifies as red/blue
- `MonitorCameraForColor` - Monitors Pi camera for target color detection
- `WaitForColorSensorClear` - Waits for color sensor to clear (offload confirmation)
- `Turn180Degrees` - Executes in-place 180° rotation
- `ReverseDistance` - Reverses robot by specified distance (default 4cm)
- `ActivateStepperMotor` - Controls stepper motor for offloading
- `StopRobot` - Ensures robot is fully stopped

### 2. **cube_delivery_mission.py**
Main behavior tree that orchestrates the complete mission:
- `MoveAndMonitorCamera` - Composite behavior for parallel navigation + camera monitoring
- `create_cube_delivery_tree()` - Builds the complete behavior tree
- `main()` - ROS2 entry point with initialization

---

## 🔧 Setup Instructions

### Prerequisites

Ensure the following ROS2 nodes are running:

```bash
# Terminal 1: Arduino Bridge (sensors, motors, color sensor)
ros2 launch ros_arduino_bridge arduino_bridge.launch.py

# Terminal 2: SLAM/Localization
ros2 launch slam_toolbox online_async_launch.py  # OR
ros2 launch nav2_bringup localization_launch.py map:=your_map.yaml

# Terminal 3: Nav2 Navigation
ros2 launch nav2_bringup navigation_launch.py

# Terminal 4: Pi Camera Node (on Raspberry Pi)
ros2 run rpi_camera_package rpicam_node

# Terminal 5: Color Detection Node (on Laptop)
ros2 run rpi_camera_package color_detection_node
```

### Coordinate Configuration

⚠️ **IMPORTANT**: Update coordinates in `cube_delivery_mission.py` to match your map!

```python
# Line ~85: Point 1 (pickup location)
move_to_point1 = MoveToPosition(
    "MoveToPickup_Point1",
    target_x=2.1,  # ← UPDATE THIS
    target_y=0.0,  # ← UPDATE THIS
    tolerance=0.2
)

# Line ~95: Point 2 (delivery location)
move_and_monitor = MoveAndMonitorCamera(
    "MoveToDelivery_Point2",
    target_x=3.0,  # ← UPDATE THIS
    target_y=1.5,  # ← UPDATE THIS
    tolerance=0.3
)
```

**How to find coordinates:**
1. Open RViz: `rviz2`
2. Load your map
3. Use "2D Goal Pose" tool to click on desired locations
4. Note the coordinates shown in the terminal/status bar
5. Update the code with those coordinates

---

## 🚀 Running the Mission

### Method 1: Direct Python Execution

```bash
# Navigate to behavior tree directory
cd ~/first_pytrees_clone/ros_arduino_bridge/behavior_tree

# Make executable
chmod +x cube_delivery_mission.py

# Run the mission
python3 cube_delivery_mission.py
```

### Method 2: ROS2 Launch File (Recommended)

Create a launch file:

```bash
# Create launch file
nano ~/first_pytrees_clone/ros_arduino_bridge/launch/cube_delivery.launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to behavior tree script
    bt_script = os.path.join(
        get_package_share_directory('ros_arduino_bridge'),
        'behavior_tree',
        'cube_delivery_mission.py'
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', bt_script],
            output='screen',
            name='cube_delivery_mission'
        )
    ])
```

Then run:
```bash
ros2 launch ros_arduino_bridge cube_delivery.launch.py
```

---

## 📊 Topic Monitoring

Monitor the mission progress with these commands:

```bash
# Monitor detected cube color
ros2 topic echo /color_sensor/rgb

# Monitor camera detection
ros2 topic echo /color_detection/detected

# Monitor robot position
ros2 topic echo /amcl_pose

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Monitor stepper motor
ros2 topic echo /stepper/debug

# Monitor robot velocity
ros2 topic echo /cmd_vel
```

---

## 🧪 Testing & Validation

### Step-by-Step Testing

1. **Test Color Sensor**
   ```bash
   # Turn on LED and read sensor
   ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"
   ros2 topic echo /color_sensor/rgb
   
   # Should show RGB values
   # Red cube: r > 0.3, r > g, r > b
   # Blue cube: b > 0.3, b > r, b > g
   ```

2. **Test Camera Detection**
   ```bash
   # Check camera is publishing
   ros2 topic hz /camera/image_raw/compressed
   
   # Check color detection
   ros2 topic echo /color_detection/detected
   
   # Place red/blue object in front of camera
   # Should publish: data: 'red' or data: 'blue'
   ```

3. **Test Navigation**
   ```bash
   # Test movement to Point 1
   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 2.1, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

4. **Test Stepper Motor**
   ```bash
   # Test offload command
   ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '-25:400:0'"
   
   # Monitor status
   ros2 topic echo /stepper/active
   ros2 topic echo /stepper/debug
   ```

5. **Test Turn and Reverse**
   ```bash
   # Manually test turning (angular velocity for ~6.3 seconds = 180°)
   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0}, angular: {z: 0.5}}"
   
   # Stop (Ctrl+C then)
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0}, angular: {z: 0.0}}"
   
   # Test reverse
   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: -0.1}, angular: {z: 0.0}}"
   ```

---

## 🐛 Troubleshooting

### Issue: Behavior tree doesn't start

**Solution:**
- Check all prerequisite nodes are running (`ros2 node list`)
- Verify map is loaded (`ros2 topic echo /map --once`)
- Check AMCL is publishing pose (`ros2 topic hz /amcl_pose`)

### Issue: Color sensor not reading

**Solution:**
```bash
# Check Arduino connection
ros2 topic echo /raw_encoders  # Should show encoder data

# Check color sensor topic exists
ros2 topic list | grep color

# Manually test LED
ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"
```

### Issue: Camera not detecting colors

**Solution:**
- Check camera node is running: `ros2 node list | grep camera`
- Verify image stream: `ros2 topic hz /camera/image_raw/compressed`
- Check color thresholds in `color_detection_node.py` line 72-77
- Adjust lighting conditions (TCS34725 and camera need good lighting)

### Issue: Robot doesn't stop at camera detection

**Solution:**
- Verify `/color_detection/detected` topic is publishing
- Check color detection threshold (line 160-165 in `sensor_behaviors.py`)
- Increase color detection area threshold if needed
- Ensure color ranges in `color_detection_node.py` match your cubes

### Issue: Navigation fails

**Solution:**
- Check coordinates are valid on your map (use RViz 2D Goal Pose)
- Verify costmaps are clear: `ros2 topic echo /global_costmap/costmap`
- Increase tolerance if robot can't reach exact position
- Check for obstacles blocking path

### Issue: Stepper motor doesn't activate

**Solution:**
```bash
# Check Arduino is receiving commands
ros2 topic echo /stepper/debug

# Verify command format
ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '-25:400:0'"

# Check Arduino firmware supports 'q' command
# Expected format: "q rpm:distance_mm:flag"
```

---

## 🎛️ Tuning Parameters

### Color Detection Thresholds

Edit `sensor_behaviors.py` line 50-60:

```python
# Red detection
if r > g and r > b and r > 0.3:  # ← Adjust 0.3 threshold
    detected = 'red'

# Blue detection  
elif b > r and b > g and b > 0.3:  # ← Adjust 0.3 threshold
    detected = 'blue'
```

### Movement Speeds

Edit `sensor_behaviors.py`:

```python
# Turn speed (line 237)
Turn180Degrees("Turn180", angular_speed=0.5)  # ← Adjust 0.5 rad/s

# Reverse speed (line 331)
ReverseDistance("Reverse4cm", distance_cm=4.0, speed=0.1)  # ← Adjust 0.1 m/s
```

### Timeouts

```python
# Color sensor timeout (sensor_behaviors.py line 32)
self.timeout = 2.0  # seconds

# Offload wait timeout (cube_delivery_mission.py line 108)
wait_for_clear = WaitForColorSensorClear("WaitForOffload", timeout=15.0)

# Stepper wait time (cube_delivery_mission.py line 120)
wait_time=5.0  # seconds
```

### Navigation Tolerances

```python
# Point 1 tolerance (cube_delivery_mission.py line 83)
tolerance=0.2  # meters (20cm)

# Point 2 tolerance (cube_delivery_mission.py line 98)
tolerance=0.3  # meters (30cm) - larger for camera-based stop
```

---

## 📋 Mission Checklist

Before running the mission:

- [ ] All ROS2 nodes running (arduino_bridge, nav2, slam, camera, color_detection)
- [ ] Map loaded and robot localized (check /amcl_pose)
- [ ] Color sensor tested (LED turns on, RGB values published)
- [ ] Camera tested (detects red/blue objects)
- [ ] Stepper motor tested (responds to commands)
- [ ] Navigation tested (robot can move to both points)
- [ ] Coordinates updated in `cube_delivery_mission.py`
- [ ] Color detection thresholds calibrated for your cubes
- [ ] Stepper motor parameters match your hardware

---

## 🔄 Behavior Tree Structure

```
Cube Delivery Mission (Sequence)
│
├── MoveToPosition("Point1", 2.1, 0.0)
│   └── Navigate to pickup location
│
├── StopRobot(1.0s)
│   └── Stabilize before sensor reading
│
├── ReadColorSensor()
│   ├── Turn on LED
│   ├── Read RGB values
│   ├── Classify as red/blue
│   └── Store in blackboard['detected_color']
│
├── MoveAndMonitorCamera("Point2", x, y)
│   ├── MoveToPosition (runs continuously)
│   └── MonitorCameraForColor (checks for target color)
│       └── SUCCESS when camera detects color → stops navigation
│
├── StopRobot(1.0s)
│   └── Full stop at delivery location
│
├── WaitForColorSensorClear(timeout=15s)
│   ├── Monitor color sensor RGB
│   └── SUCCESS when target color clears (cube offloaded)
│
├── Turn180Degrees(0.5 rad/s)
│   └── Rotate in place for ~6.3 seconds
│
├── ReverseDistance(4cm, 0.1 m/s)
│   └── Back up 4cm in ~0.4 seconds
│
└── ActivateStepperMotor(-25 RPM, 400mm, wait=5s)
    └── Final offload mechanism
```

---

## 📦 Dependencies

Ensure these packages are installed:

```bash
# ROS2 packages
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-py-trees
sudo apt install ros-humble-py-trees-ros

# Python packages
pip3 install py-trees
pip3 install opencv-python
pip3 install numpy
```

---

## 🎥 Expected Behavior

1. **Start**: Robot at origin (0, 0)
2. **Phase 1**: Robot navigates to Point 1 (2.1, 0.0) - takes ~10-30 seconds depending on distance
3. **Phase 2**: Robot stops, LED turns on, color sensor reads cube → "Detected color = RED" (or BLUE)
4. **Phase 3**: Robot starts moving toward Point 2 while camera actively scans
5. **Phase 4**: Camera detects matching color → "Camera detected RED" → robot stops immediately
6. **Phase 5**: Robot waits (up to 15s) for color sensor to clear → "Color sensor cleared! Cube offloaded"
7. **Phase 6**: Robot rotates 180° in place
8. **Phase 7**: Robot reverses 4cm
9. **Phase 8**: Stepper motor activates for final offload
10. **Complete**: "MISSION COMPLETED SUCCESSFULLY! 🎉"

---

## 🔗 Related Files

- **Arduino firmware**: Should support commands `v` (color), `q` (stepper), `s` (servos), `m` (motors)
- **Color detection config**: `color_detection_node.py` lines 72-77 (HSV ranges)
- **Navigation params**: `config/robot_params.yaml` (speeds, tolerances)
- **Map file**: Your SLAM-generated map (`.yaml` + `.pgm`)

---

## 📝 Notes

- The behavior tree uses **Sequence** with `memory=True` to preserve state across ticks
- **Blackboard** is used to share detected color between behaviors
- **MoveAndMonitorCamera** is a custom composite behavior that runs movement and monitoring in parallel
- Color sensor uses **normalized RGB values** (0.0-1.0 range) from 16-bit TCS34725
- Stepper command format: `"rpm:distance_mm:flag"` where negative RPM = offload direction

---

## 🆘 Support

If you encounter issues:

1. Check logs: `ros2 topic echo /rosout` for error messages
2. Visualize in RViz: Load robot model, map, costmaps, and camera feed
3. Test each component individually using the testing section above
4. Verify Arduino firmware supports all required commands
5. Check wiring: TCS34725 (I2C), stepper motor, servos, encoders

---

## 🎯 Success Criteria

Mission is successful when:
- ✅ Robot navigates to Point 1 without collision
- ✅ Color sensor correctly identifies cube color (red or blue)
- ✅ Robot navigates toward Point 2 while camera monitors
- ✅ Robot stops when camera detects matching color (not at exact Point 2)
- ✅ Robot waits for manual or automatic offload (color clears)
- ✅ Robot executes 180° turn
- ✅ Robot reverses 4cm
- ✅ Stepper motor activates for final offload
- ✅ No errors or exceptions in behavior tree execution

---

## 📈 Future Enhancements

Possible improvements:
- Use odometry feedback for precise reverse distance
- Add recovery behaviors for navigation failures
- Implement more sophisticated color classification (ML model)
- Add safety zones around Point 2 for multiple cubes
- Log mission metrics (time, distance, accuracy)
- Add voice/sound feedback for mission phases
- Implement auto-retry on sensor failures

Good luck with your cube delivery mission! 🚀
