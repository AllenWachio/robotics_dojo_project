# 🗺️ COMPETITION FIELD MAP WITH COORDINATES

## Field Overview

```
                    COMPETITION FIELD 2025
    ═══════════════════════════════════════════════════════════
    ║                                                         ║
    ║   📦 LOADING BAY                                       ║
    ║   (0.044, -0.52)                                       ║
    ║   [Blue Region]                                        ║
    ║                                                         ║
    ║                                        🎯 DELIVERY      ║
    ║   🌿 DISEASE STATION ─────────────────→    ZONES       ║
    ║   (-0.17, -1.22)                          [Orange]     ║
    ║   [Green X]                                            ║
    ║                                          🔴 Red Zone   ║
    ║                                          (1.326, -1.02)║
    ║   🏁 START                                             ║
    ║   (-0.17, -1.92)                         🔵 Blue Zone  ║
    ║   [Grey X]                               (1.326, -1.42)║
    ║                                                         ║
    ║                                          🟢 Green Zone ║
    ║                                          (1.326, -1.82)║
    ║                                                         ║
    ═══════════════════════════════════════════════════════════
         ← West                                    East →
         ↓ South                                   North ↑
```

## Coordinate System

### Map Frame

- **Origin:** (-0.474, -2.22, 0) meters
- **X-axis:** Points East (→)
- **Y-axis:** Points North (↑)
- **Z-axis:** Points Up (out of page)

### Robot Reference

- **Forward:** +X direction (East)
- **Left:** +Y direction (North)
- **Right:** -Y direction (South)

## Waypoint Details

### 1. 🏁 START (Grey X)

```
Position: (-0.17, -1.92)
Physical: 300mm from left, 300mm from bottom
Purpose:  Robot starting position
Action:   Set initial pose here in RViz
```

### 2. 🌿 DISEASE STATION (Green X)

```
Position: (-0.17, -1.22)
Physical: 300mm from left, 1000mm from bottom
Purpose:  Plant inspection for disease detection
Action:   Navigate here, activate camera, wait for ML result
```

### 3. 📦 LOADING BAY (Blue Region)

```
Position: (0.044, -0.52)
Physical: 518mm from left, 1700mm from bottom
Purpose:  Pick up cargo cube
Action:   Navigate close, reverse in, read RGB sensor
```

### 4. 🎯 MAZE ENTRANCE

```
Position: (0.726, -1.22)
Physical: 1200mm from left, 1000mm from bottom
Purpose:  Entry point to delivery zone path
Action:   Navigate through maze to appropriate delivery
```

### 5. 🔴 RED DELIVERY ZONE (Orange Region)

```
Position: (1.326, -1.02)
Physical: 1800mm from left, 1200mm from bottom
Purpose:  Deliver red cargo cubes
Action:   Navigate if cargo is red, reverse, offload
```

### 6. 🔵 BLUE DELIVERY ZONE (Orange Region)

```
Position: (1.326, -1.42)
Physical: 1800mm from left, 800mm from bottom
Purpose:  Deliver blue cargo cubes
Action:   Navigate if cargo is blue, reverse, offload
```

### 7. 🟢 GREEN DELIVERY ZONE (Orange Region)

```
Position: (1.326, -1.82)
Physical: 1800mm from left, 400mm from bottom
Purpose:  Deliver green cargo cubes
Action:   Navigate if cargo is green, reverse, offload
```

## Mission Path Example

### Scenario: Robot has RED cargo

```
Path Visualization:
═══════════════════════════════════════════════════════════

1. START 🏁 (-0.17, -1.92)
   ↓ [Navigate]

2. DISEASE STATION 🌿 (-0.17, -1.22)
   ↓ [Inspect plant, return]

3. START 🏁 (-0.17, -1.92)
   ↓ [Navigate]

4. LOADING BAY 📦 (0.044, -0.52)
   ↓ [Reverse in, read sensor: RED]

5. START 🏁 (-0.17, -1.92)
   ↓ [Navigate through maze]

6. RED DELIVERY 🔴 (1.326, -1.02)
   ↓ [Reverse in, offload]

7. MISSION COMPLETE ✅

═══════════════════════════════════════════════════════════
```

## Distance Calculations

### From Start to Each Waypoint

| From → To          | ΔX (m) | ΔY (m) | Distance (m) |
| ------------------ | ------ | ------ | ------------ |
| Start → Disease    | 0.00   | 0.70   | **0.70**     |
| Start → Loading    | 0.21   | 1.40   | **1.42**     |
| Start → Maze Entry | 0.90   | 0.70   | **1.14**     |
| Start → Red Zone   | 1.50   | 0.90   | **1.75**     |
| Start → Blue Zone  | 1.50   | 0.50   | **1.58**     |
| Start → Green Zone | 1.50   | 0.10   | **1.50**     |

### Between Mission Waypoints

| From → To       | Distance (m) | Time Est. (0.3 m/s) |
| --------------- | ------------ | ------------------- |
| Loading → Red   | 1.71         | ~5.7 sec            |
| Loading → Blue  | 1.48         | ~4.9 sec            |
| Loading → Green | 1.52         | ~5.1 sec            |
| Disease → Start | 0.70         | ~2.3 sec            |

## Tolerances

### Navigation Tolerances

```python
# From competition_mission.py
MoveToPosition("ToWaypoint", x, y, tolerance=0.2)
```

- **Standard waypoints:** 0.2m (20cm)
- **Delivery zones:** 0.3m (30cm) - allows some error before reverse
- **Disease station:** 0.2m (20cm) - need accurate camera view

### Why These Tolerances?

- Robot width: ~30cm
- Need wiggle room for obstacles
- Nav2 re-plans if tolerance too tight
- Delivery zones have larger area

## RViz Verification Commands

### Set Initial Pose

```bash
# In RViz, use "2D Pose Estimate" tool
# Click at: (-0.17, -1.92)
# Drag arrow to face East (→)
```

### Send Test Navigation Goals

```bash
# In RViz, use "2D Nav Goal" tool
# Click each waypoint to test paths

# Test 1: Disease Station
# Click: (-0.17, -1.22)

# Test 2: Loading Bay
# Click: (0.044, -0.52)

# Test 3: Red Delivery
# Click: (1.326, -1.02)

# Verify paths avoid walls!
```

### Terminal Monitoring

```bash
# Monitor current position
ros2 topic echo /amcl_pose

# Monitor navigation goals
ros2 topic echo /goal_pose

# Monitor robot movement
ros2 topic echo /cmd_vel
```

## Field Dimensions Reference

### Full Field

- **Width:** 2400mm (2.4m)
- **Height:** 2011mm (2.011m)
- **Area:** 4.83 m²

### Map Parameters

```yaml
# From gamefield.yaml
resolution: 0.05 # meters/pixel
origin: [-0.474, -2.22, 0] # [x, y, theta]
```

### Conversion Formulas

```python
# Physical (mm) to Pixel
pixel_x = physical_x_mm / 50.0
pixel_y = physical_y_mm / 50.0

# Pixel to Map (meters)
map_x = origin_x + (pixel_x * resolution)
map_y = origin_y + (pixel_y * resolution)

# Direct: Physical (mm) to Map (m)
map_x = origin_x + (physical_x_mm / 1000.0)
map_y = origin_y + (physical_y_mm / 1000.0)
```

## Troubleshooting Waypoints

### Robot doesn't reach waypoint

```bash
# Check tolerance
# In competition_mission.py:
MoveToPosition("Name", x, y, tolerance=0.3)  # Increase tolerance

# Check for obstacles
rviz2  # View costmap, see if path blocked
```

### Robot stops before waypoint

```bash
# Nav2 might see phantom obstacles
# Check costmap inflation radius
# View in RViz: Add → By Topic → /local_costmap/costmap
```

### Wrong waypoint reached

```bash
# Verify coordinates
python3 -c "from competition_mission import WAYPOINTS; print(WAYPOINTS)"

# Check AMCL localization
ros2 topic echo /amcl_pose  # Is robot where it thinks it is?
```

### Path goes through walls

```bash
# Check map file
rviz2  # Add → By Topic → /map
# Verify walls match physical field

# Adjust waypoints away from walls
# Keep 0.3m clearance minimum
```

## Quick Reference Card

```
╔════════════════════════════════════════════════════╗
║          WAYPOINT QUICK REFERENCE                  ║
╠════════════════════════════════════════════════════╣
║ Start:          (-0.17, -1.92)  🏁 Grey X         ║
║ Disease:        (-0.17, -1.22)  🌿 Green X        ║
║ Loading:        ( 0.04, -0.52)  📦 Blue Region    ║
║ Maze Entry:     ( 0.73, -1.22)  🚪 Midpoint       ║
║ Red Delivery:   ( 1.33, -1.02)  🔴 Orange/Red     ║
║ Blue Delivery:  ( 1.33, -1.42)  🔵 Orange/Blue    ║
║ Green Delivery: ( 1.33, -1.82)  🟢 Orange/Green   ║
╠════════════════════════════════════════════════════╣
║ Tolerance: 0.2m standard, 0.3m delivery zones     ║
║ Coordinate System: +X=East, +Y=North              ║
║ Origin: (-0.474, -2.22, 0)                        ║
╚════════════════════════════════════════════════════╝
```

**Print this for competition day! 📋**

## Testing Checklist

- [ ] Load map in RViz: `rviz2`
- [ ] Set initial pose at start: (-0.17, -1.92)
- [ ] Send nav goal to disease station: (-0.17, -1.22)
- [ ] Send nav goal to loading bay: (0.044, -0.52)
- [ ] Send nav goal to red delivery: (1.326, -1.02)
- [ ] Send nav goal to blue delivery: (1.326, -1.42)
- [ ] Send nav goal to green delivery: (1.326, -1.82)
- [ ] Verify all paths avoid walls
- [ ] Check path timing (total < 3 min)
- [ ] Adjust waypoints if needed

**Once all checked → Ready for full mission! 🚀**

---

**See also:**

- `PYTREES_COMPARISON.md` - How waypoints were calculated
- `UPDATE_SUMMARY.md` - What changed and why
- `QUICK_START.md` - Competition day commands
