# 🚀 IncrementalMove - Quick Reference

## ⚡ Quick Start

```bash
# Run test mission
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./06_incremental_test.sh
```

## 📝 Basic Usage

```python
from incremental_move_behavior import IncrementalMove

# Simple movement
move = IncrementalMove("GoToGoal", goal_x=2.0, goal_y=1.5)

# With custom parameters
move = IncrementalMove(
    name="CustomMove",
    goal_x=3.0,
    goal_y=2.0,
    step_size=0.2,      # Distance between waypoints
    tolerance=0.15,     # Goal acceptance radius
    stuck_timeout=30.0, # Seconds before declaring stuck
    max_retries=3       # Maximum recovery attempts
)
```

## 🎯 Common Configurations

### Precise Docking

```python
IncrementalMove("Dock", x, y, step_size=0.05, tolerance=0.02)
```

### Fast Traversal

```python
IncrementalMove("Cross", x, y, step_size=0.5, tolerance=0.3)
```

### Standard Navigation

```python
IncrementalMove("Navigate", x, y)  # Uses defaults
```

## 🔄 Coexist with MoveToPosition

```python
from robot_navigation_bt import MoveToPosition
from incremental_move_behavior import IncrementalMove

root = Sequence("Mission")

# Use IncrementalMove for complex navigation
complex_move = IncrementalMove("Complex", 3.0, 2.0)

# Use MoveToPosition for simple moves
simple_move = MoveToPosition("Simple", 1.0, 1.0)

root.add_children([complex_move, simple_move])
```

## 📊 What You Get

### Real-time Logs

```
[GoToGoal] ✓ Waypoint 3:
  Position: (1.237, 0.842)
  Progress: 45.2%
  Remaining: 0.743m
  Time: 5.2s
```

### Final Statistics

```
Total Time:          12.34s
Waypoints Reached:   8
Distance Traveled:   1.482m
Path Efficiency:     91.5%
Average Speed:       0.120m/s
```

## 🛠️ Requirements

- ✅ Nav2 running
- ✅ AMCL active (`/amcl_pose`)
- ✅ LiDAR publishing (`/scan`)
- ✅ Robot localized

## 🔧 Parameter Guide

| Parameter         | Small            | Standard | Large           |
| ----------------- | ---------------- | -------- | --------------- |
| **step_size**     | 0.1m (precise)   | 0.2m     | 0.5m (fast)     |
| **tolerance**     | 0.05m (exact)    | 0.15m    | 0.3m (rough)    |
| **stuck_timeout** | 15s (quick fail) | 30s      | 60s (patient)   |
| **max_retries**   | 1 (fail fast)    | 3        | 5+ (persistent) |

## 🎮 Game Field Integration

```bash
# 1. Measure waypoints on physical field (mm)
# 2. Convert to ROS coordinates
python3 waypoint_converter_interactive.py

# 3. Use in behavior tree
move = IncrementalMove("Waypoint1", 0.726, -1.220)  # Converted coords
```

## 📁 Files

| File                             | Purpose            |
| -------------------------------- | ------------------ |
| `incremental_movement_utils.py`  | Utility classes    |
| `incremental_move_behavior.py`   | Main behavior      |
| `test_incremental_navigation.py` | Test mission       |
| `06_incremental_test.sh`         | Run test           |
| `INCREMENTAL_MOVE_GUIDE.md`      | Full documentation |

## 🆘 Troubleshooting

### Robot doesn't move

```bash
ros2 topic echo /amcl_pose  # Check localization
ros2 action list | grep navigate_to_pose  # Check Nav2
```

### Stuck immediately

```python
# Increase timeout
move = IncrementalMove("Move", x, y, stuck_timeout=60.0)
```

### No obstacle avoidance

```bash
ros2 topic echo /scan --once  # Verify LiDAR
```

### Import errors

```bash
export PYTHONPATH="${PYTHONPATH}:${HOME}/ros2_ws/src/ros_arduino_bridge/behavior_tree"
```

## 💡 Tips

1. **Test in RViz first**: Use "2D Goal Pose" to verify goals are reachable
2. **Start with defaults**: step_size=0.2m works well for most cases
3. **Watch the logs**: They tell you exactly what's happening
4. **Tune iteratively**: Adjust one parameter at a time
5. **Mix behaviors**: Use IncrementalMove AND MoveToPosition!

## 🎯 When to Use What

| Situation                          | Use This          |
| ---------------------------------- | ----------------- |
| Long distance, complex environment | `IncrementalMove` |
| Need progress monitoring           | `IncrementalMove` |
| Obstacles possible                 | `IncrementalMove` |
| Simple direct movement             | `MoveToPosition`  |
| Short distance                     | `MoveToPosition`  |
| Performance critical               | `MoveToPosition`  |

## 📚 Full Documentation

For detailed info, see:

- `INCREMENTAL_MOVE_GUIDE.md` - Complete guide
- `IMPLEMENTATION_SUMMARY.md` - Technical details

---

**Ready to navigate! 🚀**
