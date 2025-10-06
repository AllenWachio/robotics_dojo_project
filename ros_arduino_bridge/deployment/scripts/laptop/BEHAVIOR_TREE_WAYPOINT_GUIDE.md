# 🎯 How to Set Waypoints for Behavior Tree

## ⚠️ Problem: "Navigation aborted!" or "Failed to create plan"

This means the waypoint coordinates are **unreachable**. Common causes:

1. **Outside map boundaries** - Coordinate is beyond the map edges
2. **Inside an obstacle** - Coordinate is in a wall, table, etc.
3. **Too close to obstacles** - Robot can't fit through
4. **Wrong frame** - Using wrong coordinate system

---

## 🔍 How to Find Valid Waypoints

### **Method 1: Using RViz (Recommended)**

1. **Open RViz** (should be running from navigation script)

2. **Click "2D Goal Pose"** tool (top toolbar)

3. **Click on an open area** on the map where you want the robot to go

4. **Watch the terminal** where navigation is running - it will print:

   ```
   Received goal: x=1.234, y=5.678
   ```

5. **Write down those coordinates**

6. **Repeat** for all waypoints you want

---

### **Method 2: Check Mouse Coordinates**

1. In RViz, hover your mouse over the map

2. Look at the **bottom status bar** of RViz

3. It shows: `x: 1.234 y: 5.678 z: 0.000`

4. These are the coordinates at your mouse position

5. Pick coordinates in **open areas** (white/gray on the map)

---

### **Method 3: Use Small Test Coordinates**

For initial testing, use small coordinates near origin:

```python
# Start at (0, 0) - your robot's starting position
# Go forward 0.5m: (0.5, 0.0)
# Go right 0.5m: (0.5, 0.5)
# Return to start: (0.0, 0.0)
```

---

## ✏️ How to Edit Waypoints

### **Step 1: Open the behavior tree file**

```bash
nano ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py
```

Or use VS Code:

```bash
code ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py
```

### **Step 2: Find the `create_root()` function**

Look for this section (around line 252):

```python
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # CHANGE THESE COORDINATES:
    move1 = MoveToPosition("MoveToPoint1", 0.5, 0.5, tolerance=0.2)
    task1 = PrintHello("Task1")
    move2 = MoveToPosition("MoveToPoint2", 0.5, -0.5, tolerance=0.2)
    task2 = PrintHi("Task2")

    root.add_children([move1, task1, move2, task2])
    return root
```

### **Step 3: Replace with YOUR coordinates**

Example for your gamefield map:

```python
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # Game field patrol mission
    move1 = MoveToPosition("Corner1", 1.0, 1.0, tolerance=0.2)  # Top-left corner
    task1 = PrintHello("Task1")

    move2 = MoveToPosition("Corner2", 1.0, -1.0, tolerance=0.2)  # Bottom-left
    task2 = PrintHi("Task2")

    move3 = MoveToPosition("Corner3", -1.0, -1.0, tolerance=0.2)  # Bottom-right

    move4 = MoveToPosition("Home", 0.0, 0.0, tolerance=0.2)  # Return home

    root.add_children([move1, task1, move2, task2, move3, move4])
    return root
```

### **Step 4: Save and test**

```bash
# No need to rebuild! Just run again:
./05_behavior_tree_mission.sh
```

---

## 📊 Understanding Coordinates

### **Coordinate System:**

```
        Y (forward)
        ↑
        |
        |
        |
←-------+-------→ X (right)
        |
        |
        |
```

- **Origin (0, 0)**: Where you set initial pose
- **Positive X**: Robot's right side
- **Positive Y**: Robot's forward direction
- **Units**: Meters

### **Example Waypoints:**

```python
(0.0, 0.0)    # Starting position
(1.0, 0.0)    # 1 meter to the right
(0.0, 1.0)    # 1 meter forward
(-1.0, 0.0)   # 1 meter to the left
(0.0, -1.0)   # 1 meter backward
(1.0, 1.0)    # 1m right + 1m forward (diagonal)
```

---

## 🎯 Tips for Choosing Good Waypoints

### ✅ **DO:**

- Choose points in **open areas** (white/light gray on map)
- Leave at least **0.5m clearance** from walls
- Test with **short distances** first (0.5m - 1.0m)
- Use **even numbers** for easier debugging (1.0, 2.0, etc.)
- Start **close to robot's position**

### ❌ **DON'T:**

- Don't pick points in **black areas** (walls/obstacles)
- Don't pick points **outside the map**
- Don't use **huge distances** for first test (> 5m)
- Don't pick points **in narrow corridors** (robot might get stuck)

---

## 🧪 Test Sequence

### **Test 1: Tiny Movement**

```python
move1 = MoveToPosition("Test1", 0.2, 0.0, tolerance=0.2)  # Just 20cm right
```

**Goal:** Verify system works with minimal movement

### **Test 2: Forward/Backward**

```python
move1 = MoveToPosition("Forward", 0.0, 0.5, tolerance=0.2)   # 50cm forward
move2 = MoveToPosition("Back", 0.0, 0.0, tolerance=0.2)      # Return
```

**Goal:** Test forward/backward navigation

### **Test 3: Square Pattern**

```python
move1 = MoveToPosition("P1", 1.0, 0.0, tolerance=0.2)
move2 = MoveToPosition("P2", 1.0, 1.0, tolerance=0.2)
move3 = MoveToPosition("P3", 0.0, 1.0, tolerance=0.2)
move4 = MoveToPosition("P4", 0.0, 0.0, tolerance=0.2)  # Home
```

**Goal:** Test full autonomous patrol

---

## 🔧 Troubleshooting

### **Error: "Planning algorithm failed to generate a valid path"**

**Cause:** Waypoint is unreachable

**Solution:**

1. Open RViz
2. Look at the map
3. Find the red "X" or arrow (failed goal)
4. Check if it's inside an obstacle
5. Move waypoint to an open area

---

### **Error: "Robot position (0.00, 0.00)"**

**Cause:** Initial pose not set

**Solution:**

1. In RViz, click "2D Pose Estimate"
2. Click where robot actually is
3. Drag to set orientation
4. Wait 2-3 seconds for localization
5. Run behavior tree again

---

### **Robot spins in place**

**Cause:** Waypoint very close to current position

**Solution:**

- Increase distance to at least 0.5m
- Or increase `tolerance` parameter

---

### **Navigation keeps aborting**

**Cause:** Path is blocked or waypoint unreachable

**Solution:**

1. Check RViz costmap (red = obstacle)
2. Pick waypoint farther from obstacles
3. Clear costmap: `ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty`

---

## 📝 Quick Reference

```python
# Basic structure
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # Add as many waypoints as you want:
    wp1 = MoveToPosition("Name1", x, y, tolerance=0.2)
    task1 = PrintHello("Task1")  # Or your custom behavior

    wp2 = MoveToPosition("Name2", x, y, tolerance=0.2)
    task2 = PrintHi("Task2")

    # Add to sequence (executes in order):
    root.add_children([wp1, task1, wp2, task2])
    return root
```

---

## 🎓 Examples for Different Maps

### **Small Room (2m x 2m)**

```python
move1 = MoveToPosition("Center", 1.0, 1.0, tolerance=0.2)
move2 = MoveToPosition("Home", 0.0, 0.0, tolerance=0.2)
```

### **Hallway**

```python
move1 = MoveToPosition("Point1", 0.0, 2.0, tolerance=0.2)   # 2m forward
move2 = MoveToPosition("Point2", 0.0, 4.0, tolerance=0.2)   # 4m forward
move3 = MoveToPosition("Return", 0.0, 0.0, tolerance=0.2)   # Back to start
```

### **Game Field (large area)**

```python
# Patrol perimeter
move1 = MoveToPosition("TopLeft", 2.0, 2.0, tolerance=0.2)
move2 = MoveToPosition("TopRight", 2.0, -2.0, tolerance=0.2)
move3 = MoveToPosition("BottomRight", -2.0, -2.0, tolerance=0.2)
move4 = MoveToPosition("BottomLeft", -2.0, 2.0, tolerance=0.2)
move5 = MoveToPosition("Home", 0.0, 0.0, tolerance=0.2)
```

---

## ✅ Summary Checklist

Before running behavior tree:

- [ ] Navigation is running
- [ ] Initial pose is set in RViz
- [ ] Localization is stable (small covariance ellipse)
- [ ] Waypoints are in **open areas** on map
- [ ] Waypoints are **reachable** (test with 2D Goal Pose first)
- [ ] Distances are reasonable (0.5m - 3m for testing)
- [ ] Robot has clear path to waypoints

---

**Now edit your waypoints and try again!** 🤖🎯
