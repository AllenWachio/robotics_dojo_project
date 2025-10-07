# Behavior Tree - Py_trees State Machine

## Overview
This folder contains the **exact copy** of the py_trees behavior tree state machine from the `gazebo_ignition_fortress` project.

## File
- **`robot_navigation_bt.py`**: Complete behavior tree implementation (342 lines, exact copy)

## Source
Copied from: `/root/ros2_ws/src/gazebo_ignition_fortress/test_folder/app.py`

## What It Does
This behavior tree implements autonomous navigation with task sequencing:

```
1. Navigate to waypoint (2.1, 0.0)
2. Execute task: Print "hello"
3. Navigate to waypoint (0, -1.2)
4. Execute task: Print "hi"
```

## How to Run

### Quick Start
```bash
# Terminal 1: Launch robot with Nav2 navigation
ros2 launch ros_arduino_bridge full_slam_test.launch.py

# Terminal 2: Run behavior tree
python3 robot_navigation_bt.py
```

### Or use the launch file:
```bash
ros2 launch ros_arduino_bridge behavior_tree_navigation.launch.py
```

## Requirements
- Nav2 navigation stack running
- AMCL localization (or SLAM)
- Map loaded (if using localization)
- Python packages: `py_trees`, `py_trees_ros`

## Key Features
✅ **Exact state machine** - No modifications to original logic  
✅ **Nav2 integration** - Uses NavigateToPose action  
✅ **AMCL localization** - Tracks robot pose  
✅ **Implicit FSM** - 7-state machine within behaviors  
✅ **Tick-based execution** - Non-blocking async operation  

## Customization
Edit waypoints and tasks in the `create_root()` function to match your robot's mission requirements.

## Documentation
See `../PYTREES_INTEGRATION_GUIDE.md` for complete documentation.

---
*State machine preserved from gazebo_ignition_fortress project*
