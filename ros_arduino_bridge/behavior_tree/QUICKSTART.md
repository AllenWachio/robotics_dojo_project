# Cube Pickup and Delivery Mission - Quick Start

## 🎯 What This Does

Your robot will:
1. Navigate to Point 1 (2.1, 0.0)
2. Read cube color (red or blue) with TCS34725 sensor
3. Navigate to Point 2 while camera searches for matching color
4. Stop when camera detects the color
5. Wait for cube offload (color sensor clears)
6. Turn 180°, reverse 4cm, activate stepper motor

---

## 🚀 Quick Start (3 Steps)

### 1. Update Coordinates

Edit `cube_delivery_mission.py` lines 83 and 95:

```python
# Point 1: Where to pick up cube
target_x=2.1,  # ← YOUR COORDINATE
target_y=0.0,  # ← YOUR COORDINATE

# Point 2: Where to deliver cube  
target_x=3.0,  # ← YOUR COORDINATE
target_y=1.5,  # ← YOUR COORDINATE
```

**Find coordinates:** Use RViz "2D Goal Pose" tool to click on your map.

### 2. Test Components

```bash
cd ~/first_pytrees_clone/ros_arduino_bridge/behavior_tree

# Test color sensor
python3 test_mission_components.py color

# Test camera
python3 test_mission_components.py camera

# Test all (includes movement!)
python3 test_mission_components.py all
```

### 3. Run Mission

Make sure these are running first:
```bash
# Terminal 1
ros2 launch ros_arduino_bridge arduino_bridge.launch.py

# Terminal 2  
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3
ros2 run rpi_camera_package rpicam_node

# Terminal 4
ros2 run rpi_camera_package color_detection_node

# Terminal 5 - RUN THE MISSION
python3 cube_delivery_mission.py
```

---

## 📊 Monitor Progress

```bash
# Watch cube color detection
ros2 topic echo /color_sensor/rgb

# Watch camera detection
ros2 topic echo /color_detection/detected

# Watch robot position
ros2 topic echo /odom
```

---

## 🐛 Common Issues

### "No color data received"
- Check Arduino is connected: `ros2 topic echo /raw_encoders`
- Turn on LED manually: `ros2 topic pub --once /color_sensor/led std_msgs/msg/Bool "data: true"`

### "Camera not detecting colors"
- Check camera is running: `ros2 topic hz /camera/image_raw/compressed`
- Check lighting (needs bright light)
- Adjust color thresholds in `color_detection_node.py` line 72-77

### "Navigation failed"
- Verify coordinates are valid in RViz
- Check map is loaded: `ros2 topic echo /map --once`
- Increase tolerance in `cube_delivery_mission.py`

### "Stepper not working"
- Test manually: `ros2 topic pub --once /stepper/command std_msgs/msg/String "data: '-25:400:0'"`
- Check debug: `ros2 topic echo /stepper/debug`

---

## 📁 Files

- **cube_delivery_mission.py** - Main behavior tree (run this!)
- **sensor_behaviors.py** - All sensor/motor behaviors
- **test_mission_components.py** - Test individual components
- **CUBE_DELIVERY_GUIDE.md** - Complete documentation

---

## 🎛️ Tuning

### Color Detection Sensitivity

Edit `sensor_behaviors.py` line 54:

```python
if r > g and r > b and r > 0.3:  # ← Lower = more sensitive
```

### Movement Speed

Edit `sensor_behaviors.py`:

```python
# Turn speed (line 237)
angular_speed=0.5  # ← rad/s

# Reverse speed (line 331)  
speed=0.1  # ← m/s
```

### Timeouts

Edit `cube_delivery_mission.py`:

```python
# Offload wait (line 108)
timeout=15.0  # ← seconds

# Stepper wait (line 120)
wait_time=5.0  # ← seconds
```

---

## ✅ Mission Success Criteria

- ✅ Robot reaches Point 1
- ✅ Color sensor reads cube (red or blue)
- ✅ Robot moves toward Point 2
- ✅ Camera detects matching color → robot stops
- ✅ Color sensor clears (cube offloaded)
- ✅ Robot turns 180°
- ✅ Robot reverses 4cm
- ✅ Stepper motor activates

---

## 🆘 Help

See **CUBE_DELIVERY_GUIDE.md** for:
- Detailed troubleshooting
- Complete parameter reference
- Arduino firmware requirements
- Advanced tuning options

---

## 🔗 Architecture

```
cube_delivery_mission.py
├── sensor_behaviors.py
│   ├── ReadColorSensor (/color_sensor/rgb)
│   ├── MonitorCameraForColor (/color_detection/detected)
│   ├── WaitForColorSensorClear
│   ├── Turn180Degrees (/cmd_vel)
│   ├── ReverseDistance (/cmd_vel)
│   └── ActivateStepperMotor (/stepper/command)
│
└── robot_navigation_bt.py
    └── MoveToPosition (Nav2 action client)
```

**Blackboard Data:**
- `detected_color` - Stores 'red' or 'blue' from color sensor

---

Good luck! 🚀
