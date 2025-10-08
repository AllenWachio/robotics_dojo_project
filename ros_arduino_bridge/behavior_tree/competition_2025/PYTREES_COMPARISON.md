# PyTrees Implementation Comparison

## Reference Code vs Our Implementation

### What We Learned from Reference Code ✅

#### 1. **Shared AMCL Subscription Pattern**

**Reference Code:**

```python
class MoveToPosition(py_trees.behaviour.Behaviour):
    # Shared class variables
    global_x = 0.0
    global_y = 0.0
    global_yaw = 0.0
    pose_initialized = False
    pose_sub = None  # Single subscription shared across instances

    @classmethod
    def shared_pose_callback(cls, msg):
        cls.global_x = msg.pose.pose.position.x
        cls.global_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        cls.pose_initialized = True
```

**Our Code:** ✅ **ADOPTED**

- We use the exact same pattern in `behaviors/navigation.py`
- All `MoveToPosition` instances share one AMCL subscription
- More efficient than creating subscription per instance

---

#### 2. **Timer-Based Tree Execution**

**Reference Code:**

```python
tree_completed = False

def tick_tree():
    nonlocal tree_completed

    if tree_completed:
        return

    node.tick_tock(period_ms=500)  # Execute tree regularly

    if node.root.status == py_trees.common.Status.SUCCESS:
        print("Behavior tree completed successfully!")
        tree_completed = True

timer = node.node.create_timer(0.5, tick_tree)  # Tick every 0.5s
rclpy.spin(node.node)
```

**Our Code:** ✅ **ADOPTED**

- We use identical timer-based execution
- Regular 500ms ticks for responsive behavior
- Completion tracking prevents repeated execution

---

#### 3. **Initial Pose Publication**

**Reference Code:**

```python
# Publish initial pose to AMCL (set robot's starting position)
initial_pose_pub = node.node.create_publisher(
    PoseWithCovarianceStamped, '/initialpose', 10)

time.sleep(1.0)  # Wait for publisher

initial_pose = PoseWithCovarianceStamped()
initial_pose.header.frame_id = 'map'
initial_pose.pose.pose.position.x = 0.0  # Start at origin
initial_pose.pose.pose.position.y = 0.0
initial_pose.pose.pose.orientation.w = 1.0

for _ in range(5):  # Publish multiple times
    initial_pose_pub.publish(initial_pose)
    time.sleep(0.1)
```

**Our Code:** ✅ **ADOPTED + IMPROVED**

- We publish initial pose at ACTUAL field start position
- Uses `WAYPOINTS['start']` = (-0.17, -1.92) from field image
- More accurate than assuming origin

---

### What We Added Beyond Reference 🚀

#### 1. **Complex Mission Structure**

**Reference:** Simple 2-waypoint navigation

```python
move1 = MoveToPosition("Point1", 2.1, 0.0)
task1 = PrintHello()
move2 = MoveToPosition("Point2", 0, -1.2)
task2 = PrintHi()
```

**Ours:** 4-phase competition mission

```python
root.add_children([
    disease_detection_phase,  # Optional plant inspection
    cargo_loading_phase,      # Navigate, reverse, read color
    maze_navigation_phase,    # Navigate based on color
    cargo_delivery_phase,     # Reverse, offload with fallback
])
```

---

#### 2. **Advanced Composites**

**Reference:** Only Sequence

```python
root = py_trees.composites.Sequence("RootSequence", memory=True)
```

**Ours:** Sequence + Selector + Parallel

```python
# Selector for color-based branching
selector = py_trees.composites.Selector("ChooseDelivery", memory=False)

# Parallel for navigation + monitoring
parallel = py_trees.composites.Parallel(
    "NavigateAndMonitor",
    policy=py_trees.common.ParallelPolicy.SuccessOnOne()
)
```

---

#### 3. **Sensor Integration**

**Reference:** No sensors

```python
# Just navigation + print
```

**Ours:** Full sensor suite

```python
ReadColorSensor("ReadCargoColor", timeout=10.0)
MonitorCameraForColor("MonitorRed", target_color='red')
WaitForDiseaseDetection("DetectDisease", timeout=15.0)
```

---

#### 4. **Actuator Control**

**Reference:** No actuators

```python
# Movement only
```

**Ours:** Multiple actuators with fallback

```python
offload_fallback = py_trees.composites.Selector("OffloadFallback", memory=False)

# Try conveyor first
try_conveyor = ActivateConveyorBelt("TryConveyor", duration=3.0)

# If fails, try conveyor + tipper
tipper_sequence = py_trees.composites.Sequence("ConveyorPlusTipper", memory=True)
activate_tipper = ActivateTipperServo("TiltRobot", angle=45)
conveyor_with_tilt = ActivateConveyorBelt("ConveyorWithTilt", duration=3.0)
reset_tipper = ResetTipperServo("ResetTilt")
```

---

#### 5. **Split Architecture Support**

**Reference:** Single machine

```python
# All runs on one computer
```

**Ours:** Distributed Pi + Laptop

```bash
# On Pi (hardware)
./01_pi_hardware.sh  # Arduino, LiDAR, Camera

# On Laptop (processing)
./02_laptop_processing.sh  # Nav2, Vision, RViz
./03_run_mission.sh  # Behavior tree
```

---

## Waypoint Extraction from Field Image 📐

### Field Analysis

- **Dimensions:** 2400mm × 2011mm
- **Map resolution:** 0.05m/pixel
- **Map origin:** (-0.474, -2.22, 0)

### Conversion Formula

```
map_x = origin_x + (pixel_x_mm / 1000)
map_y = origin_y + (pixel_y_mm / 1000)
```

### Extracted Waypoints

| Location              | Field Position | Pixel Coords     | Map Coords         |
| --------------------- | -------------- | ---------------- | ------------------ |
| **Grey X (Start)**    | Bottom-left    | (300mm, 300mm)   | **(-0.17, -1.92)** |
| **Green X (Disease)** | Left middle    | (300mm, 1000mm)  | **(-0.17, -1.22)** |
| **Blue (Loading)**    | Top-left       | (518mm, 1700mm)  | **(0.044, -0.52)** |
| **Orange-Red**        | Right upper    | (1800mm, 1200mm) | **(1.326, -1.02)** |
| **Orange-Blue**       | Right middle   | (1800mm, 800mm)  | **(1.326, -1.42)** |
| **Orange-Green**      | Right lower    | (1800mm, 400mm)  | **(1.326, -1.82)** |

### Visual Map

```
        ↑ Y (North)
        |
   [Loading Bay]
        |    (Blue region)
        |
   [Disease] ───────────────→ [Delivery Zones]
        |    (Green X)           (Orange region)
        |                         - Red
   [Start]                        - Blue
    (Grey X)                      - Green
        |
        └──────────────────────→ X (East)
```

---

## Key Improvements Made 🔧

### 1. **Accurate Initial Pose**

```python
# Before: Assumed origin
initial_pose.pose.pose.position.x = 0.0
initial_pose.pose.pose.position.y = 0.0

# After: Use actual field start
initial_pose.pose.pose.position.x = WAYPOINTS['start'][0]  # -0.17
initial_pose.pose.pose.position.y = WAYPOINTS['start'][1]  # -1.92
```

### 2. **Real Field Coordinates**

```python
# Before: Guessed waypoints
'loading_bay': (0.3, 0.8)

# After: Measured from field image
'loading_bay': (0.044, -0.52)  # Based on blue region at 518mm, 1700mm
```

### 3. **Added Documentation**

Every waypoint now has:

- Physical field position (mm)
- Pixel calculation
- Final map coordinates
- Visual explanation

---

## What Makes Our Implementation Better ⭐

| Aspect                 | Reference Code         | Our Implementation                |
| ---------------------- | ---------------------- | --------------------------------- |
| **AMCL Integration**   | ✅ Shared subscription | ✅ Same pattern                   |
| **Timer Execution**    | ✅ 500ms ticks         | ✅ Same pattern                   |
| **Initial Pose**       | ✅ Published           | ✅ + Accurate position            |
| **Mission Complexity** | ❌ 2 waypoints         | ✅ 7 waypoints, 4 phases          |
| **Composites Used**    | ❌ Sequence only       | ✅ Sequence + Selector + Parallel |
| **Sensor Integration** | ❌ None                | ✅ RGB, Camera, Disease ML        |
| **Actuator Control**   | ❌ None                | ✅ Conveyor, Tipper, Camera servo |
| **Fallback Logic**     | ❌ None                | ✅ Selector-based retries         |
| **Split Architecture** | ❌ Single machine      | ✅ Pi + Laptop distributed        |
| **Field Accuracy**     | ❌ Generic coords      | ✅ Measured from field image      |

---

## Testing the New Waypoints 🧪

### 1. Visual Verification

```bash
# On laptop
rviz2 -d /path/to/config.rviz

# In RViz:
# - Set initial pose at (-0.17, -1.92)
# - Send 2D Nav Goal to each waypoint
# - Verify paths make sense
```

### 2. Waypoint Test Script

```python
# Run from competition_2025/
python3 -c "
from competition_mission import WAYPOINTS
for name, (x, y) in WAYPOINTS.items():
    print(f'{name:20s}: ({x:6.3f}, {y:6.3f})')
"
```

### 3. Full Mission Dry Run

```bash
# Terminal 1 (Pi)
./launch/01_pi_hardware.sh

# Terminal 2 (Laptop)
./launch/02_laptop_processing.sh

# Terminal 3 (Laptop - after map loads)
./launch/03_run_mission.sh
```

---

## Summary

✅ **Adopted from Reference:**

- Shared AMCL subscription pattern
- Timer-based tree execution
- Initial pose publication

🚀 **Enhanced Beyond Reference:**

- Complex 4-phase mission structure
- Multiple composite types (Sequence, Selector, Parallel)
- Full sensor and actuator integration
- Distributed Pi+Laptop architecture
- Accurate field-measured waypoints
- Fallback and retry logic

📐 **Waypoint Accuracy:**

- All coordinates extracted from actual field image
- Documented with physical measurements
- Ready for real-world testing

The reference code taught us clean patterns for basic navigation. We built on that foundation to create a complete autonomous competition system! 🏆
