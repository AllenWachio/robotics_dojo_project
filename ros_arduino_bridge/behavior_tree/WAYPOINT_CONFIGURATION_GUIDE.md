# Waypoint Configuration Guide for Autonomous Navigation
**Date:** October 7, 2025  
**Mission:** Disease Detection + Cube Pickup & Delivery

---

## ❓ Do You Need to Add Waypoints?

### **Short Answer: YES!** ✅

You need to configure the waypoint coordinates in `cube_delivery_mission.py` to match your **actual physical environment**. The current coordinates are **placeholder examples**.

---

## 📍 Current Waypoint Status

### ⚠️ **Lines That MUST Be Updated:**

#### **Line 207-212: Point 1 (Pickup Location)**
```python
move_to_point1 = MoveToPosition(
    "MoveToPickup_Point1",
    target_x=2.1,  # ⚠️ UPDATE THIS!
    target_y=0.0,  # ⚠️ UPDATE THIS!
    tolerance=0.2
)
```

#### **Line 225-230: Point 2 (Delivery Location)**
```python
move_and_monitor = MoveAndMonitorCamera(
    "MoveToDelivery_Point2",
    target_x=3.0,  # ⚠️ UPDATE THESE!
    target_y=1.5,  # ⚠️ UPDATE THESE!
    tolerance=0.3
)
```

#### **✅ Already Configured (Don't Change):**
- **Origin:** `(0.0, 0.0)` - Robot's starting position
- **Return to Origin:** `(0.0, 0.0)` - After disease detection

---

## 🗺️ How Navigation Works

### **Your Mission Uses Nav2 (ROS2 Navigation Stack)**

```
1. Robot starts at origin (0, 0)
2. AMCL localization tracks robot position in map frame
3. Nav2 plans path from current position → target waypoint
4. Robot navigates autonomously avoiding obstacles
5. Stops when within tolerance distance of target
```

### **No Manual Path Planning Needed!**
- Nav2 handles obstacle avoidance automatically
- Just provide (x, y) coordinates in **map frame**
- Robot figures out the best path

---

## 🛠️ How to Determine Your Waypoints

### **Method 1: Using RViz (Recommended) 🎯**

#### **Step 1: Launch Your Robot with Navigation**
```bash
# Terminal 1: Launch complete robot system
ros2 launch ros_arduino_bridge complete_robot.launch.py
```

#### **Step 2: Open RViz for Visualization**
```bash
# Terminal 2: Launch RViz
rviz2
```

#### **Step 3: Configure RViz Display**
1. **Add Map Display:**
   - Click "Add" → "By topic" → `/map` → `Map`
   
2. **Add Robot Model:**
   - Click "Add" → "RobotModel"
   
3. **Add AMCL Pose:**
   - Click "Add" → "By topic" → `/amcl_pose` → `PoseWithCovarianceStamped`

4. **Set Fixed Frame:**
   - In "Global Options" → Fixed Frame: `map`

#### **Step 4: Record Coordinates**

**For Pickup Location (Point 1):**
```bash
# Drive robot to pickup location manually
ros2 topic echo /amcl_pose --once

# Output example:
# pose:
#   position:
#     x: 2.1
#     y: 0.0
#     z: 0.0

# ✅ Use these coordinates in line 209-210!
```

**For Delivery Location (Point 2):**
```bash
# Drive robot to delivery location manually
ros2 topic echo /amcl_pose --once

# Output example:
# pose:
#   position:
#     x: 3.0
#     y: 1.5
#     z: 0.0

# ✅ Use these coordinates in line 227-228!
```

---

### **Method 2: Using Nav2 "2D Pose Estimate" Tool 📌**

#### **In RViz:**
1. Click **"2D Pose Estimate"** button (top toolbar)
2. Click on map where pickup location should be
3. Drag to set orientation
4. Terminal will show coordinates:
   ```
   [INFO]: Setting estimate pose: 2.1, 0.0
   ```
5. **Write down these coordinates!**
6. Repeat for delivery location

---

### **Method 3: Measure Physical Distances 📏**

If you know your robot's physical environment:

```
Origin (0, 0) = Robot starting position

Point 1 (Pickup):
├─ x = Distance forward from origin (meters)
└─ y = Distance left(+)/right(-) from origin (meters)

Point 2 (Delivery):
├─ x = Distance forward from origin (meters)
└─ y = Distance left(+)/right(-) from origin (meters)
```

**Example Layout:**
```
         Y (left/right)
         ↑
         |
    [P2] | (3.0, 1.5)   ← Delivery location
         |
    [P1] | (2.1, 0.0)   ← Pickup location
         |
    [●]--+----------→ X (forward/backward)
   Origin
   (0,0)
```

---

## 📝 Step-by-Step Configuration

### **1. Determine Your Coordinates** (Use Method 1 or 2 above)

Let's say you found:
- **Pickup:** `x = 1.5, y = -0.3`
- **Delivery:** `x = 2.8, y = 1.2`

### **2. Update cube_delivery_mission.py**

Open the file and modify these sections:

```python
# Line 207-212: Update Point 1 (Pickup)
move_to_point1 = MoveToPosition(
    "MoveToPickup_Point1",
    target_x=1.5,   # YOUR measured X
    target_y=-0.3,  # YOUR measured Y
    tolerance=0.2   # Stop within 20cm
)
```

```python
# Line 225-230: Update Point 2 (Delivery)
move_and_monitor = MoveAndMonitorCamera(
    "MoveToDelivery_Point2",
    target_x=2.8,   # YOUR measured X
    target_y=1.2,   # YOUR measured Y
    tolerance=0.3   # Stop within 30cm
)
```

### **3. Adjust Tolerance Values (Optional)**

**What is tolerance?**
- Distance (in meters) within which robot considers goal "reached"
- Smaller = more precise, but may take longer
- Larger = faster, but less precise positioning

**Recommended Values:**
```python
tolerance=0.1  # Very precise (10cm) - for tight spaces
tolerance=0.2  # Precise (20cm) - default for pickup
tolerance=0.3  # Moderate (30cm) - default for delivery
tolerance=0.5  # Loose (50cm) - for rough positioning
```

---

## 🧪 Testing Your Waypoints

### **Test Each Waypoint Individually:**

```python
# Create test script: test_single_waypoint.py
import rclpy
from robot_navigation_bt import MoveToPosition

def test_waypoint(x, y):
    rclpy.init()
    
    behavior = MoveToPosition("TestWaypoint", target_x=x, target_y=y)
    # Setup and run behavior...
    
    print(f"Testing waypoint: ({x}, {y})")

# Test pickup location
test_waypoint(1.5, -0.3)

# Test delivery location
test_waypoint(2.8, 1.2)
```

### **Run with Visualization:**
```bash
# Terminal 1: Launch robot
ros2 launch ros_arduino_bridge complete_robot.launch.py

# Terminal 2: Run test
python3 test_single_waypoint.py

# Terminal 3: Monitor in RViz
rviz2
```

---

## 🚨 Common Issues & Solutions

### **Issue 1: Robot Doesn't Move**
**Cause:** Nav2 action server not ready  
**Solution:**
```bash
# Check if action server is running
ros2 action list

# Should see: /navigate_to_pose
```

### **Issue 2: "Goal Failed" Message**
**Cause:** Target unreachable (obstacle, off-map, etc.)  
**Solution:**
- Check waypoint is within map boundaries
- Ensure path is clear of obstacles
- Increase tolerance value

### **Issue 3: Robot Stops Too Far From Target**
**Cause:** Tolerance too large  
**Solution:**
```python
tolerance=0.1  # Decrease tolerance for more precision
```

### **Issue 4: Robot Overshoots Target**
**Cause:** Controller parameters too aggressive  
**Solution:**
- Tune Nav2 controller parameters in `nav2_params.yaml`
- Or increase tolerance temporarily

---

## 📊 Complete Mission Waypoint Summary

| Phase | Waypoint | Current Coords | Action Required | Description |
|-------|----------|----------------|-----------------|-------------|
| 0.1 | Relative Move | `(1.22m, -0.15m)` | ✅ OK | Disease detection position |
| 0.6 | Origin | `(0.0, 0.0)` | ✅ OK | Return home after detection |
| 1 | Point 1 (Pickup) | `(2.1, 0.0)` | ⚠️ **UPDATE!** | Cube pickup location |
| 4 | Point 2 (Delivery) | `(3.0, 1.5)` | ⚠️ **UPDATE!** | Cube delivery location |

---

## 🎯 Quick Configuration Checklist

- [ ] **Step 1:** Launch robot and Nav2 navigation
- [ ] **Step 2:** Drive robot to pickup location manually
- [ ] **Step 3:** Record coordinates from `/amcl_pose`
- [ ] **Step 4:** Update `cube_delivery_mission.py` line 209-210
- [ ] **Step 5:** Drive robot to delivery location
- [ ] **Step 6:** Record coordinates from `/amcl_pose`
- [ ] **Step 7:** Update `cube_delivery_mission.py` line 227-228
- [ ] **Step 8:** Test each waypoint individually
- [ ] **Step 9:** Run complete mission and verify

---

## 📖 Related Documentation

- **Nav2 Documentation:** https://navigation.ros.org/
- **AMCL Localization:** `ros_arduino_bridge/README.md`
- **Complete Mission Guide:** `COMPLETE_MISSION_GUIDE.md`
- **Testing Guide:** `test_mission_components.py`

---

## 🔗 Helpful Commands

```bash
# Check current robot position
ros2 topic echo /amcl_pose --once

# Send test navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}}"

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Visualize map and robot
rviz2
```

---

**Summary:** Yes, you need to configure waypoints! Use RViz + `/amcl_pose` topic to find your pickup and delivery coordinates, then update lines 209-210 and 227-228 in `cube_delivery_mission.py`.

**Next Step:** Launch your robot system, drive to each location manually, record the coordinates, and update the file! 🚀
