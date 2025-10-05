# Py_trees Behavior Tree Integration Guide

## Overview

This package now includes a **py_trees-based behavior tree state machine** for autonomous robot navigation, copied exactly from the `gazebo_ignition_fortress` project. The state machine provides high-level task orchestration for your robotics_dojo_project robot.

## What Was Added

### 1. **Behavior Tree State Machine**
- **Location**: `behavior_tree/robot_navigation_bt.py`
- **Source**: Exact copy of `gazebo_ignition_fortress/test_folder/app.py`
- **Purpose**: Autonomous navigation with task sequencing

### 2. **Dependencies Added to package.xml**
```xml
<!-- Navigation2 dependencies for behavior tree -->
<depend>nav2_msgs</depend>
<depend>navigation2</depend>
<depend>nav2_bringup</depend>

<!-- Py_trees dependencies for behavior tree state machine -->
<exec_depend>python3-py-trees</exec_depend>
<exec_depend>python3-py-trees-ros</exec_depend>
```

### 3. **Launch File**
- **Location**: `launch/behavior_tree_navigation.launch.py`
- **Purpose**: Launches the behavior tree for autonomous operation

## Behavior Tree Structure

The state machine implements this exact sequence:

```
RootSequence (Sequential execution with memory)
├── MoveToPoint1: Navigate to (2.1, 0.0)
├── Task1: Print "hello"
├── MoveToPoint2: Navigate to (0, -1.2)
└── Task2: Print "hi"
```

### How It Works

1. **MoveToPosition Behavior** (Complex state machine within a behavior):
   - Subscribes to AMCL pose (`/amcl_pose`)
   - Sets navigation tolerance dynamically
   - Sends NavigateToPose action to Nav2
   - Monitors navigation progress
   - Returns SUCCESS when goal reached

2. **PrintHello/PrintHi Behaviors** (Simple actions):
   - Execute once and return SUCCESS
   - Demonstrate task execution between navigation waypoints

3. **Tick Mechanism**:
   - Tree is ticked every 500ms
   - Each behavior returns: `RUNNING`, `SUCCESS`, or `FAILURE`
   - Sequence progresses to next child on SUCCESS

## Installation Requirements

### 1. Install Python Dependencies
```bash
pip3 install py_trees py_trees_ros
```

### 2. Install ROS2 Dependencies
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-msgs
sudo apt install python3-py-trees python3-py-trees-ros
```

### 3. Rebuild the Package
```bash
cd /root/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash
```

## How to Use

### Method 1: Using Launch File (Recommended)

**Step 1: Launch your robot with Nav2**
You need Nav2 navigation stack running. This typically includes:
- Robot base (Arduino bridge)
- SLAM or localization (AMCL)
- Nav2 navigation stack
- Map (if using localization mode)

Example:
```bash
# Terminal 1: Launch robot and navigation
ros2 launch ros_arduino_bridge full_slam_test.launch.py
```

**Step 2: Launch behavior tree**
```bash
# Terminal 2: Launch py_trees behavior tree
ros2 launch ros_arduino_bridge behavior_tree_navigation.launch.py
```

### Method 2: Running Directly

```bash
# Make sure Nav2 is running first
python3 /root/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py
```

## Expected Behavior

When you run the behavior tree, you should see:

```
Initial pose published to AMCL
Behavior tree starting...
MoveToPoint1: Waiting for AMCL pose data...
MoveToPoint1: Set goal tolerance to 0.2m
MoveToPoint1: Sending goal to (2.1, 0.0)
MoveToPoint1: Goal accepted, navigating...
MoveToPoint1: Current(0.00, 0.00) -> Target(2.1, 0.0), Distance: 2.10m
MoveToPoint1: Current(0.52, 0.03) -> Target(2.1, 0.0), Distance: 1.59m
...
MoveToPoint1: Navigation completed! Final distance: 0.18m
hello
MoveToPoint2: Sending goal to (0, -1.2)
MoveToPoint2: Goal accepted, navigating...
...
hi
Behavior tree completed successfully!
```

## Customization

### Change Navigation Waypoints

Edit `behavior_tree/robot_navigation_bt.py`:

```python
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)
    
    # Modify these coordinates for your use case
    move1 = MoveToPosition("MoveToPoint1", 2.1, 0.0, tolerance=0.2)
    task1 = PrintHello("Task1")
    move2 = MoveToPosition("MoveToPoint2", 0, -1.2, tolerance=0.2)
    task2 = PrintHi("Task2")
    
    # Add more waypoints
    move3 = MoveToPosition("MoveToPoint3", 1.5, 1.5, tolerance=0.2)
    
    root.add_children([move1, task1, move2, task2, move3])
    return root
```

### Add Custom Behaviors

Create new behavior classes inheriting from `py_trees.behaviour.Behaviour`:

```python
class MyCustomBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name="MyBehavior"):
        super().__init__(name)
        # Initialize state
    
    def setup(self, **kwargs):
        # Setup ROS2 nodes, subscribers, etc.
        node = kwargs.get('node')
        # Create subscriptions, action clients, etc.
    
    def update(self):
        # Implement behavior logic
        # Return: py_trees.common.Status.SUCCESS
        #         py_trees.common.Status.FAILURE
        #         py_trees.common.Status.RUNNING
        pass
```

## State Machine Architecture

### Layer 1: Py_trees (High-level)
```
Behavior Tree → Sequential task execution
```

### Layer 2: Nav2 BT Navigator (Mid-level)
```
NavigateToPose Action → Path planning → Path following
```

### Layer 3: Robot Control (Low-level)
```
cmd_vel → Arduino Bridge → Motor control
```

## Key Classes

### 1. MoveToPosition
**Purpose**: Navigate robot to target coordinates  
**State Machine**: 7-state implicit FSM  
**Actions**: 
- Sets navigation tolerance
- Sends Nav2 goals
- Monitors progress
- Returns SUCCESS/FAILURE

### 2. PrintHello / PrintHi
**Purpose**: Simple task execution  
**Actions**: Print message once, return SUCCESS

### 3. Behavior Tree Root
**Type**: `py_trees.composites.Sequence` with `memory=True`  
**Behavior**: Execute children in order, stop on failure

## Integration with Your Robot

### Required Topics/Services
Your robot system must provide:

1. **AMCL Localization**: `/amcl_pose` (PoseWithCovarianceStamped)
2. **Nav2 Action**: `/navigate_to_pose` (NavigateToPose action)
3. **Initial Pose**: `/initialpose` (PoseWithCovarianceStamped publisher)
4. **Controller Server**: `/controller_server/set_parameters` (for tolerance setting)

### Map Requirement
If using localization mode (recommended):
- Pre-created map loaded in Nav2
- AMCL running with the map
- Robot's initial pose set

If using SLAM mode:
- SLAM Toolbox running
- Real-time mapping enabled

## Troubleshooting

### "No module named 'py_trees'"
```bash
pip3 install py_trees py_trees_ros
```

### "NavigateToPose action server not available"
Nav2 is not running. Launch your navigation stack first:
```bash
ros2 launch nav2_bringup navigation_launch.py
```

### "Waiting for AMCL pose data..."
AMCL is not publishing poses. Check:
```bash
ros2 topic echo /amcl_pose
```
If no output, ensure:
- AMCL node is running
- Map is loaded
- Initial pose is set

### Robot doesn't move
Check cmd_vel is being published:
```bash
ros2 topic echo /cmd_vel
# or
ros2 topic echo /diff_drive_base_controller/cmd_vel_unstamped
```

### Goal rejected
Nav2 controller might be having issues. Check:
```bash
ros2 topic echo /navigate_to_pose/_action/feedback
```

## Monitoring and Debugging

### View Behavior Tree Status
```bash
# Install py_trees viewer (optional)
pip3 install py_trees[ros]
py_trees-tree-viewer
```

### Monitor ROS2 Topics
```bash
# Navigation goals
ros2 topic echo /navigate_to_pose/_action/goal

# Robot pose
ros2 topic echo /amcl_pose

# Velocity commands
ros2 topic echo /cmd_vel
```

### RViz2 Visualization
- Robot pose in map frame
- Navigation path (green line)
- Costmaps (local and global)
- LiDAR scan data

## Advanced: Composite Node Types

The py_trees framework supports multiple composite types:

### Sequence
```python
# Executes children in order, stops on first FAILURE
sequence = py_trees.composites.Sequence("MySequence", memory=True)
```

### Selector (Fallback)
```python
# Executes children in order, stops on first SUCCESS
selector = py_trees.composites.Selector("MySelector")
```

### Parallel
```python
# Executes all children simultaneously
parallel = py_trees.composites.Parallel("MyParallel")
```

## Integration Summary

✅ **Exact state machine copied** from gazebo_ignition_fortress  
✅ **Zero modifications** to the core logic  
✅ **Dependencies added** to package.xml  
✅ **Launch file created** for easy startup  
✅ **Documentation provided** for usage  

The py_trees behavior tree is now fully integrated into your robotics_dojo_project and ready to use for autonomous navigation!

## Next Steps

1. **Test the integration**: Launch your robot and run the behavior tree
2. **Customize waypoints**: Edit coordinates for your specific use case
3. **Add behaviors**: Create custom behaviors for your robot's tasks
4. **Integrate sensors**: Add sensor-based decision making to behaviors
5. **Build complex trees**: Use Selectors, Parallels for more sophisticated logic

## References

- **Original Source**: `/root/ros2_ws/src/gazebo_ignition_fortress/test_folder/app.py`
- **Py_trees Docs**: https://py-trees.readthedocs.io/
- **Nav2 Docs**: https://navigation.ros.org/
- **Behavior Trees in Robotics**: Colledanchise & Ögren (2018)

---

*Integration completed on October 5, 2025*
