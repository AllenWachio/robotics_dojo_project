# 🚀 IncrementalMove Behavior - Complete Guide

## Overview

The **IncrementalMove** behavior is an advanced py_trees behavior that provides incremental navigation with obstacle avoidance and detailed progress monitoring. It **coexists** with the traditional `MoveToPosition` behavior - use whichever suits your needs!

## 📋 Table of Contents

1. [Quick Start](#quick-start)
2. [When to Use](#when-to-use)
3. [Features](#features)
4. [Basic Usage](#basic-usage)
5. [Advanced Configuration](#advanced-configuration)
6. [Integration Examples](#integration-examples)
7. [Obstacle Recovery](#obstacle-recovery)
8. [Logging & Statistics](#logging--statistics)
9. [Troubleshooting](#troubleshooting)
10. [API Reference](#api-reference)

---

## Quick Start

### Run the Test Mission

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./06_incremental_test.sh
```

### Minimum Requirements

- ✅ Navigation stack running (Nav2)
- ✅ AMCL localization active
- ✅ LiDAR publishing to `/scan`
- ✅ Robot localized on map

---

## When to Use

### Use **IncrementalMove** when you need:

| Feature                 | IncrementalMove           | MoveToPosition    |
| ----------------------- | ------------------------- | ----------------- |
| **Progress monitoring** | ✅ Every 0.2m             | ❌ Only start/end |
| **Obstacle avoidance**  | ✅ LiDAR-based recovery   | ❌ None           |
| **Detailed logging**    | ✅ Full statistics        | ❌ Basic status   |
| **Stuck detection**     | ✅ With retry logic       | ❌ None           |
| **Movement analysis**   | ✅ Path efficiency, speed | ❌ None           |
| **Simplicity**          | ⚠️ More complex           | ✅ Simple         |
| **Performance**         | ⚠️ More overhead          | ✅ Lightweight    |

### Decision Guide

**Use IncrementalMove for:**

- Long-distance navigation in complex environments
- Missions requiring progress feedback
- Environments with dynamic obstacles
- Tasks needing precise movement tracking
- Development and debugging (detailed logs)

**Use MoveToPosition for:**

- Simple direct movements
- Short distances in open spaces
- When performance is critical
- Quick prototyping

**Both can coexist in the same behavior tree!**

---

## Features

### 1. **Incremental Waypoints**

- Breaks long movements into small steps (default 0.2m)
- Each step is a separate navigation goal
- Progress logged at every waypoint

### 2. **Obstacle Avoidance**

- Uses LiDAR data from `/scan` topic
- Detects blocked paths automatically
- Generates evasive waypoints around obstacles
- Finds clear directions dynamically

### 3. **Stuck Detection & Recovery**

- Monitors movement progress continuously
- Detects when robot is stuck (no progress for timeout)
- Automatic retry with alternative routes (up to 3 attempts)
- Exponential backoff between retries

### 4. **Detailed Logging**

- Position at each waypoint
- Distance traveled per step
- Progress percentage
- Time elapsed
- Movement statistics summary

### 5. **Map-Agnostic Design**

- Works with any map coordinate system
- No hardcoded coordinates
- Pure vector mathematics
- Easy to port to different environments

---

## Basic Usage

### Import the Behavior

```python
from incremental_move_behavior import IncrementalMove
```

### Simple Movement

```python
# Move to a position with default settings
move = IncrementalMove(
    name="GoToKitchen",
    goal_x=3.5,
    goal_y=2.1
)
```

### Add to Behavior Tree

```python
def create_mission():
    root = py_trees.composites.Sequence("Mission", memory=True)

    # Use IncrementalMove
    move1 = IncrementalMove("MoveToA", 2.0, 1.5)

    # Use traditional MoveToPosition (they coexist!)
    move2 = MoveToPosition("MoveToB", 1.0, 0.5)

    root.add_children([move1, move2])
    return root
```

---

## Advanced Configuration

### All Parameters

```python
move = IncrementalMove(
    name="CustomMove",
    goal_x=3.0,                # Target X (meters, map frame)
    goal_y=2.0,                # Target Y (meters, map frame)
    step_size=0.2,             # Distance between waypoints (meters)
    tolerance=0.15,            # Goal acceptance radius (meters)
    stuck_timeout=30.0,        # Seconds before declaring stuck
    max_retries=3              # Maximum recovery attempts
)
```

### Parameter Tuning Guide

#### **step_size** (Default: 0.2m)

| Value    | Use Case                         | Trade-off               |
| -------- | -------------------------------- | ----------------------- |
| **0.1m** | Precise docking, tight spaces    | Slower, more waypoints  |
| **0.2m** | General navigation (recommended) | Balanced                |
| **0.5m** | Long distances in open areas     | Faster, less monitoring |

```python
# Example: Precise docking
precise_move = IncrementalMove("Dock", 1.0, 0.5, step_size=0.1)

# Example: Fast traversal
fast_move = IncrementalMove("CrossRoom", 5.0, 3.0, step_size=0.5)
```

#### **tolerance** (Default: 0.15m)

| Value     | Use Case                                 |
| --------- | ---------------------------------------- |
| **0.05m** | Precise positioning (docking, alignment) |
| **0.15m** | General navigation (recommended)         |
| **0.30m** | Rough positioning, large spaces          |

```python
# Example: Precise task
precise = IncrementalMove("Align", 2.0, 1.0, tolerance=0.05)

# Example: Rough positioning
rough = IncrementalMove("NearTable", 3.0, 2.0, tolerance=0.30)
```

#### **stuck_timeout** (Default: 30.0s)

| Value     | Use Case                         |
| --------- | -------------------------------- |
| **15.0s** | Fast environments, quick failure |
| **30.0s** | Standard (recommended)           |
| **60.0s** | Slow robot, difficult terrain    |

```python
# Example: Patient robot on rough terrain
patient = IncrementalMove("Difficult", 2.0, 1.0, stuck_timeout=60.0)
```

#### **max_retries** (Default: 3)

| Value | Use Case                         |
| ----- | -------------------------------- |
| **1** | Fail fast, no recovery           |
| **3** | Standard (recommended)           |
| **5** | Persistent, complex environments |

```python
# Example: Persistent navigation
persistent = IncrementalMove("Complex", 3.0, 2.0, max_retries=5)
```

---

## Integration Examples

### Example 1: Room Mapping Mission

```python
def create_mapping_mission():
    """Map room and visit inspection points."""
    root = py_trees.composites.Sequence("MapRoom", memory=True)

    # Phase 1: Map the room (traditional move)
    explore1 = MoveToPosition("Corner1", 2.0, 2.0)
    explore2 = MoveToPosition("Corner2", 2.0, -2.0)
    explore3 = MoveToPosition("Corner3", -2.0, -2.0)
    explore4 = MoveToPosition("Corner4", -2.0, 2.0)

    # Phase 2: Visit inspection points (incremental for precision)
    inspect1 = IncrementalMove("InspectA", 1.0, 1.0, step_size=0.15)
    task1 = PerformTask("TakePhoto1")

    inspect2 = IncrementalMove("InspectB", -1.0, 1.0, step_size=0.15)
    task2 = PerformTask("TakePhoto2")

    # Return home (traditional - simple)
    return_home = MoveToPosition("Home", 0.0, 0.0)

    root.add_children([
        explore1, explore2, explore3, explore4,
        inspect1, task1,
        inspect2, task2,
        return_home
    ])
    return root
```

### Example 2: Object Delivery

```python
def create_delivery_mission():
    """Pick up object and deliver with obstacle avoidance."""
    root = py_trees.composites.Sequence("Delivery", memory=True)

    # Go to pickup location (incremental for safety)
    go_to_pickup = IncrementalMove(
        "GoToPickup",
        goal_x=3.0,
        goal_y=1.5,
        step_size=0.2,
        max_retries=5  # Persistent navigation
    )

    # Precise alignment for pickup
    align_pickup = IncrementalMove(
        "AlignForPickup",
        goal_x=3.1,
        goal_y=1.5,
        step_size=0.05,   # Small steps
        tolerance=0.03    # Very precise
    )

    pickup_task = PerformTask("PickupObject", duration=3.0)

    # Navigate to delivery location
    go_to_delivery = IncrementalMove(
        "GoToDelivery",
        goal_x=-2.0,
        goal_y=-1.0,
        step_size=0.2
    )

    # Precise alignment for delivery
    align_delivery = IncrementalMove(
        "AlignForDelivery",
        goal_x=-2.1,
        goal_y=-1.0,
        step_size=0.05,
        tolerance=0.03
    )

    delivery_task = PerformTask("DeliverObject", duration=2.0)

    # Return quickly (traditional move)
    return_home = MoveToPosition("ReturnHome", 0.0, 0.0)

    root.add_children([
        go_to_pickup,
        align_pickup,
        pickup_task,
        go_to_delivery,
        align_delivery,
        delivery_task,
        return_home
    ])
    return root
```

### Example 3: Patrol Mission

```python
def create_patrol_mission():
    """Patrol waypoints with monitoring."""
    root = py_trees.composites.Sequence("Patrol", memory=True)

    waypoints = [
        (2.0, 2.0, "Checkpoint1"),
        (2.0, -2.0, "Checkpoint2"),
        (-2.0, -2.0, "Checkpoint3"),
        (-2.0, 2.0, "Checkpoint4")
    ]

    for x, y, name in waypoints:
        # Move incrementally to monitor for anomalies
        move = IncrementalMove(
            f"MoveTo{name}",
            goal_x=x,
            goal_y=y,
            step_size=0.3  # Larger steps for patrol
        )

        # Check area
        check = PerformTask(f"Check{name}", "Scan for anomalies", duration=2.0)

        root.add_children([move, check])

    return root
```

---

## Obstacle Recovery

### How It Works

1. **Detection**: Monitors progress every tick
2. **Stuck Condition**: No movement > 0.05m for timeout period
3. **LiDAR Analysis**: Scans 360° to find clear directions
4. **Evasive Waypoint**: Generates waypoint 0.5m in clear direction
5. **Retry**: Continues to goal from new position
6. **Failure**: After max_retries, returns FAILURE

### Recovery States

```
MOVING ──[stuck detected]──> RECOVERING
    │                             │
    │                             ├──[clear path found]──> insert evasive waypoint
    │                             │                              │
    │                             │                              v
    │                             │                          MOVING
    │                             │
    │                             └──[no clear path]──> replan from current position
    │                                      │                      │
    │                                      v                      v
    │                                 PLANNING ──────────────> MOVING
    │
    └──[max retries exceeded]──> FAILED
```

### Tuning Recovery Behavior

```python
# Aggressive recovery (quick to react)
aggressive = IncrementalMove(
    "Aggressive",
    goal_x=3.0,
    goal_y=2.0,
    stuck_timeout=15.0,  # Declare stuck quickly
    max_retries=5        # Try many times
)

# Conservative recovery (patient)
conservative = IncrementalMove(
    "Conservative",
    goal_x=3.0,
    goal_y=2.0,
    stuck_timeout=60.0,  # Wait longer before declaring stuck
    max_retries=2        # Few retries
)
```

---

## Logging & Statistics

### Real-time Logs

During execution, you'll see:

```
[MoveToKitchen] ✓ Waypoint 3:
  Position: (1.237, 0.842)
  Progress: 45.2% (0.613m / 1.356m)
  Remaining: 0.743m
  Step distance: 0.203m
  Time: 5.2s
```

### Final Statistics

At completion:

```
============================================================
[MoveToKitchen] MOVEMENT SUMMARY
============================================================
Total Time:          12.34s
Waypoints Reached:   8
Distance Traveled:   1.482m
Straight Line Dist:  1.356m
Path Efficiency:     91.5%
Average Speed:       0.120m/s
============================================================
```

### Understanding Statistics

- **Total Time**: Wall clock time from start to goal
- **Waypoints Reached**: Number of intermediate waypoints hit
- **Distance Traveled**: Actual path length
- **Straight Line Dist**: Direct distance start→goal
- **Path Efficiency**: (straight/traveled) × 100% - closer to 100% is better
- **Average Speed**: traveled distance / total time

### Low Path Efficiency?

If path efficiency < 80%, possible causes:

- Obstacles forced detours
- Poor localization (wandering)
- Nav2 planner taking indirect routes
- Recovery maneuvers

---

## Troubleshooting

### Issue: Robot doesn't move

**Symptoms**: Behavior runs but robot stationary

**Checks**:

1. Is AMCL publishing pose? `ros2 topic echo /amcl_pose`
2. Is Nav2 running? `ros2 node list | grep controller`
3. Is goal reachable? Test in RViz with "2D Goal Pose"
4. Check logs for "Navigation goal rejected"

**Solution**:

```bash
# Verify navigation stack
ros2 action list | grep navigate_to_pose

# Check controller server
ros2 node info /controller_server
```

### Issue: Robot declared stuck immediately

**Symptoms**: "STUCK DETECTED!" after < 5 seconds

**Cause**: Stuck timeout too low or position not updating

**Solution**:

```python
# Increase timeout
move = IncrementalMove("Move", 2.0, 1.0, stuck_timeout=60.0)

# Verify AMCL publishing
ros2 topic hz /amcl_pose  # Should be ~1-10 Hz
```

### Issue: No obstacle avoidance

**Symptoms**: Robot stuck, no recovery attempts

**Checks**:

1. Is LiDAR publishing? `ros2 topic echo /scan --once`
2. Is `/scan` topic name correct?
3. Check ObstacleAvoidance logs

**Solution**:

```bash
# Verify LiDAR
ros2 topic info /scan
ros2 topic hz /scan  # Should be ~5-10 Hz
```

### Issue: Recovery fails repeatedly

**Symptoms**: Max retries exceeded, mission fails

**Possible Causes**:

- Goal truly unreachable (in obstacle, off map)
- LiDAR range too limited
- Evasion distance too small

**Solution**:

```python
# More persistent recovery
move = IncrementalMove(
    "Persistent",
    goal_x=3.0,
    goal_y=2.0,
    max_retries=10,  # More attempts
    stuck_timeout=45.0  # More patient
)
```

Or verify goal in RViz first!

### Issue: Import errors

**Symptoms**: `ImportError: No module named 'incremental_move_behavior'`

**Solution**:

```bash
# Ensure Python path includes behavior_tree directory
export PYTHONPATH="${PYTHONPATH}:${HOME}/ros2_ws/src/ros_arduino_bridge/behavior_tree"

# Or run from correct directory
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree
python3 test_incremental_navigation.py
```

---

## API Reference

### IncrementalMove Class

```python
class IncrementalMove(py_trees.behaviour.Behaviour):
    """
    Advanced incremental navigation behavior.
    """

    def __init__(
        self,
        name: str,
        goal_x: float,
        goal_y: float,
        step_size: float = 0.2,
        tolerance: float = 0.15,
        stuck_timeout: float = 30.0,
        max_retries: int = 3
    )
```

#### Parameters

| Parameter       | Type  | Default  | Description                             |
| --------------- | ----- | -------- | --------------------------------------- |
| `name`          | str   | required | Behavior name for identification        |
| `goal_x`        | float | required | Target X coordinate (meters, map frame) |
| `goal_y`        | float | required | Target Y coordinate (meters, map frame) |
| `step_size`     | float | 0.2      | Distance between waypoints (meters)     |
| `tolerance`     | float | 0.15     | Goal acceptance radius (meters)         |
| `stuck_timeout` | float | 30.0     | Seconds before declaring stuck          |
| `max_retries`   | int   | 3        | Maximum recovery attempts               |

#### Return Values

| Status    | Meaning                                     |
| --------- | ------------------------------------------- |
| `SUCCESS` | Goal reached within tolerance               |
| `FAILURE` | Max retries exceeded or unrecoverable error |
| `RUNNING` | Still executing                             |

#### State Machine

States: `IDLE` → `PLANNING` → `MOVING` → `CHECKING` → `GOAL_REACHED`

Recovery path: `MOVING` → `RECOVERING` → `PLANNING` → `MOVING`

### Utility Classes

#### WaypointGenerator

```python
waypoints = WaypointGenerator.generate(
    start_x, start_y, goal_x, goal_y, step_size
)
```

#### PositionTracker

```python
tracker = PositionTracker(node)
pose = tracker.get_pose()  # Returns (x, y, yaw) or None
distance = tracker.distance_to(target_x, target_y)
```

#### MovementLogger

```python
logger = MovementLogger(ros_logger, name, start_pose, goal_pose)
logger.log_waypoint(current_pose, distance_traveled)
summary = logger.get_summary()
```

#### ObstacleAvoidance

```python
avoidance = ObstacleAvoidance(node, safe_distance=0.5)
is_blocked = avoidance.is_path_blocked(angle_rad)
clear_angle = avoidance.find_clear_direction()
```

---

## Files Reference

| File                                    | Purpose                                                    |
| --------------------------------------- | ---------------------------------------------------------- |
| `incremental_movement_utils.py`         | Utility classes (WaypointGenerator, PositionTracker, etc.) |
| `incremental_move_behavior.py`          | Main IncrementalMove behavior class                        |
| `test_incremental_navigation.py`        | Test mission demonstrating usage                           |
| `test_incremental_navigation.launch.py` | ROS2 launch file                                           |
| `06_incremental_test.sh`                | Easy-to-use shell script launcher                          |
| `INCREMENTAL_MOVE_GUIDE.md`             | This documentation                                         |

---

## Summary

### Key Takeaways

1. ✅ **IncrementalMove** and **MoveToPosition** coexist - use as needed
2. ✅ Default parameters (step_size=0.2m) work well for most cases
3. ✅ Obstacle avoidance requires LiDAR on `/scan` topic
4. ✅ Detailed logs help with debugging and tuning
5. ✅ Map-agnostic design works anywhere

### Quick Decision Matrix

| Scenario                             | Recommended Behavior                                    |
| ------------------------------------ | ------------------------------------------------------- |
| Simple A→B movement                  | `MoveToPosition`                                        |
| Long distance in complex environment | `IncrementalMove`                                       |
| Need progress feedback               | `IncrementalMove`                                       |
| Precise docking/alignment            | `IncrementalMove` (small step_size)                     |
| Fast traversal in open space         | `MoveToPosition` or `IncrementalMove` (large step_size) |
| Development/debugging                | `IncrementalMove` (for logs)                            |

### Next Steps

1. **Test**: Run `./06_incremental_test.sh`
2. **Customize**: Modify waypoints in `test_incremental_navigation.py`
3. **Integrate**: Add IncrementalMove to your missions
4. **Tune**: Adjust parameters based on your robot and environment

---

**Questions? Issues? Check the troubleshooting section or review logs!** 🚀
