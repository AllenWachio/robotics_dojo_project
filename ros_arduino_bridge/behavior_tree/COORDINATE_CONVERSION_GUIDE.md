# 📐 Coordinate Conversion Guide for Game Field

## 🎯 Quick Answer to Your Question

**"How do we break the points in the map into X and Y points?"**

### **Your Map Configuration:**

```
Image size:     135 × 154 pixels
Resolution:     0.05 m/pixel (50mm per pixel)
Map size:       6.75 × 7.70 meters
Origin:         (-0.474, -2.220) meters
Game field:     2.4 × 2.011 meters (2400 × 2011 mm)
```

### **✅ Answer: Use the Coordinate Converter Tool!**

```bash
# Run the converter tool:
python3 ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/coordinate_converter.py
```

---

## 📊 Understanding the Coordinate Systems

### **1. Real-World (Physical Game Field)**

```
Origin (0,0) is at one corner of the game field

        Y↑ (2011mm)
         |
    2400 +──────────────+
      mm |              |
         |   GAME       |
         |   FIELD      |
         |              |
       0 +──────────────+→ X (2400mm)
         0              2400mm
```

**Your green, red, blue X marks:**

- Measure from physical field corner
- Units: millimeters (mm)
- Example: "1200mm from left, 1000mm from bottom"

---

### **2. ROS Map Frame (What Robot Uses)**

```
ROS coordinates are in METERS with origin offset

    Y↑
     |
  2.0+        Field area
     |     ┌─────────┐
  1.0+     │         │
     |     │         │
  0.0+─────┼─────────┼───→ X
     |     │         │
 -1.0+     │         │
     |     └─────────┘
 -2.0+   Origin: (-0.474, -2.220)
     |
    -1  0  1  2  3 (meters)
```

**For behavior tree navigation:**

- Use these ROS coordinates
- Units: meters
- Origin is offset: (-0.474, -2.220)

---

## 🔧 How to Convert Your Waypoints

### **Method 1: Using the Python Tool (RECOMMENDED)**

```python
from coordinate_converter import CoordinateConverter

# Initialize
converter = CoordinateConverter("~/ros2_ws/maps/gamefield.yaml")

# Convert real-world (mm) to ROS (meters)
x_mm, y_mm = 1200, 1000  # Your measured waypoint
x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
print(f"ROS waypoint: ({x_ros:.3f}, {y_ros:.3f}) m")

# Use in behavior tree:
move1 = MoveToPosition("WP1", x_ros, y_ros, tolerance=0.2)
```

---

### **Method 2: Manual Calculation**

**Formula:**

```
X_ros = (X_mm / 1000) + origin_x
Y_ros = (Y_mm / 1000) + origin_y

Where:
  origin_x = -0.474 meters
  origin_y = -2.220 meters
```

**Example:**

```
Real-world point: (1200mm, 1000mm)

X_ros = (1200 / 1000) + (-0.474) = 1.2 - 0.474 = 0.726 m
Y_ros = (1000 / 1000) + (-2.220) = 1.0 - 2.220 = -1.220 m

Result: (0.726, -1.220) meters
```

---

## 📍 Common Game Field Points

Based on your field dimensions (2400 × 2011 mm):

| Location             | Real-World (mm) | ROS Coordinates (m) | Description        |
| -------------------- | --------------- | ------------------- | ------------------ |
| **Origin (Green X)** | (0, 0)          | (-0.474, -2.220)    | Bottom-left corner |
| **Bottom-Right**     | (2400, 0)       | (1.926, -2.220)     | Right side start   |
| **Center**           | (1200, 1006)    | (0.726, -1.214)     | Middle of field    |
| **Top-Left**         | (0, 2011)       | (-0.474, -0.209)    | Upper left         |
| **Top-Right**        | (2400, 2011)    | (1.926, -0.209)     | Upper right corner |

---

## 🎯 Step-by-Step: Converting Your Path Points

### **Step 1: Identify Points on Physical Field**

Look at your game field diagram and mark the waypoints:

```
Example from your image:
- Green X (Start):  (0, 0) mm
- Blue X (Middle):  (1200, 1000) mm  ← You need to measure these!
- Red X (End):      (2400, 2011) mm
```

### **Step 2: Convert to ROS Coordinates**

Use the converter:

```python
# In Python or add to behavior tree
from coordinate_converter import CoordinateConverter

converter = CoordinateConverter("~/ros2_ws/maps/gamefield.yaml")

# Your waypoints from physical measurements
waypoint1_mm = (0, 0)        # Green X
waypoint2_mm = (1200, 1000)  # Blue X
waypoint3_mm = (2400, 2011)  # Red X

# Convert to ROS
wp1_ros = converter.realworld_to_ros(*waypoint1_mm)
wp2_ros = converter.realworld_to_ros(*waypoint2_mm)
wp3_ros = converter.realworld_to_ros(*waypoint3_mm)

print(f"Waypoint 1 (Green X): {wp1_ros}")
print(f"Waypoint 2 (Blue X):  {wp2_ros}")
print(f"Waypoint 3 (Red X):   {wp3_ros}")
```

### **Step 3: Use in Behavior Tree**

```python
def create_root():
    root = py_trees.composites.Sequence("GameFieldMission")

    # Use converted ROS coordinates
    move1 = MoveToPosition("ToBlueX", 0.726, -1.220, tolerance=0.2)
    task1 = PrintHello("Task1")

    move2 = MoveToPosition("ToRedX", 1.926, -0.209, tolerance=0.2)
    task2 = PrintHi("Task2")

    root.add_children([move1, task1, move2, task2])
    return root
```

---

## 🔄 Quick Reference Conversions

### **Real-World → ROS (meters)**

```python
x_ros = (x_mm / 1000.0) + (-0.474)
y_ros = (y_mm / 1000.0) + (-2.220)
```

### **ROS → Real-World (mm)**

```python
x_mm = (x_ros - (-0.474)) * 1000.0
y_mm = (y_ros - (-2.220)) * 1000.0
```

### **Example Conversions:**

```
(0, 0) mm       →  (-0.474, -2.220) m
(500, 500) mm   →  (0.026, -1.720) m
(1000, 1000) mm →  (0.526, -1.220) m
(1500, 1500) mm →  (1.026, -0.720) m
(2000, 2000) mm →  (1.526, -0.220) m
(2400, 2011) mm →  (1.926, -0.209) m
```

---

## 🛠️ Tools Provided

### **1. coordinate_converter.py**

Full-featured coordinate conversion tool:

```bash
# Run interactive demo
python3 ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/coordinate_converter.py

# Use in your code
from coordinate_converter import CoordinateConverter
converter = CoordinateConverter("~/ros2_ws/maps/gamefield.yaml")
```

**Features:**

- ✅ Real-world (mm) ↔ ROS (m) conversion
- ✅ Pixel ↔ ROS conversion
- ✅ Waypoint generation with configurable step size
- ✅ CSV export for analysis
- ✅ Batch processing

---

### **2. Interactive Waypoint Tool (Coming Next)**

Will let you:

- Click on image to get coordinates
- Visualize path on map
- Export waypoints for behavior tree
- Test conversions interactively

---

## 📝 Practical Example

### **Scenario: Navigate the Blue Path**

From your second image showing the cyan path:

**Step 1: Identify Path Points**

```
Green X (Start):  Measure from corner: (~100, 400) mm
Blue path turns at multiple points
Red X (End):      Measure from corner: (~1800, 1600) mm
```

**Step 2: Convert Path Points**

```python
converter = CoordinateConverter("~/ros2_ws/maps/gamefield.yaml")

# Main waypoints along the blue path
path_points_mm = [
    (100, 400),    # Green X start
    (300, 400),    # First segment
    (300, 800),    # Turn up
    (600, 800),    # Go right
    (600, 1200),   # Turn up again
    (1000, 1200),  # Continue right
    (1000, 1600),  # Final turn
    (1800, 1600),  # Red X end
]

# Convert all to ROS
path_points_ros = [
    converter.realworld_to_ros(x, y)
    for x, y in path_points_mm
]

# Print for behavior tree
for i, (x, y) in enumerate(path_points_ros):
    print(f"move{i+1} = MoveToPosition('WP{i+1}', {x:.3f}, {y:.3f})")
```

**Step 3: Build Behavior Tree**

```python
def create_game_field_mission():
    root = py_trees.composites.Sequence("GameFieldPatrol")

    moves = []
    for i, (x, y) in enumerate(path_points_ros):
        move = MoveToPosition(f"Waypoint{i+1}", x, y, tolerance=0.15)
        moves.append(move)

    root.add_children(moves)
    return root
```

---

## 🎨 Visualization

### **Generate Path Visualization**

```python
import matplotlib.pyplot as plt

# Convert waypoints
waypoints_ros = [converter.realworld_to_ros(x, y) for x, y in path_points_mm]

# Extract X and Y
x_coords = [wp[0] for wp in waypoints_ros]
y_coords = [wp[1] for wp in waypoints_ros]

# Plot
plt.figure(figsize=(10, 8))
plt.plot(x_coords, y_coords, 'b-', linewidth=2, label='Path')
plt.plot(x_coords[0], y_coords[0], 'go', markersize=15, label='Start (Green X)')
plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=15, label='End (Red X)')
plt.plot(x_coords[1:-1], y_coords[1:-1], 'bx', markersize=10, label='Waypoints')

plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Game Field Navigation Path')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
```

---

## ✅ Summary

### **To convert your waypoints:**

1. **Measure** real-world positions in millimeters from field corner
2. **Convert** using: `converter.realworld_to_ros(x_mm, y_mm)`
3. **Use** the ROS coordinates in your behavior tree
4. **Test** with small movements first!

### **Quick conversion formula:**

```
X_ros (meters) = (X_mm / 1000) + (-0.474)
Y_ros (meters) = (Y_mm / 1000) + (-2.220)
```

### **Files to use:**

- `coordinate_converter.py` - Conversion tool
- `gamefield.yaml` - Your map configuration
- Behavior tree - Use converted ROS coordinates

---

## 🚀 Next Steps

1. **Measure** your green/blue/red X positions on physical field
2. **Run converter** to get ROS coordinates
3. **Update** behavior tree with correct waypoints
4. **Test** navigation with real coordinates!

**Need help measuring? Let me know and I can create a measurement guide!**
