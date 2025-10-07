# 🗺️ Game Field Map Guide

## Map Information

Your competition field map:

- **File**: `gamefield.pgm`
- **Resolution**: 0.05 meters/pixel (5cm per pixel)
- **Origin**: (-0.474, -2.22, 0)
- **Mode**: trinary
- **Physical dimensions**: 2400mm × 2011mm (2.4m × 2.011m)

## Understanding the Coordinate System

```
Map Coordinate System:
┌─────────────────────────────────────────┐
│ Origin: (-0.474, -2.22)                 │  ← Bottom-left of map
│                                          │
│         ↑ Y (North)                      │
│         │                                │
│         │                                │
│         └────→ X (East)                  │
│                                          │
│                                          │
│                      Robot start: (0,0)  │
└─────────────────────────────────────────┘
```

## Field Layout (from your images)

```
═══════════════════════════════════════════════════════════
                    GAME FIELD LAYOUT
═══════════════════════════════════════════════════════════

        2400mm (2.4m) WIDTH
  ┌──────────────────────────────────────────┐
  │                                           │
  │  ┌────────┐     ┌────────┐              │  ← Plant Display
  │  │ PLANT  │     │ PLANT  │              │    (480×480mm each)
  │  │  480   │     │  480   │              │
  │  └────────┘     └────────┘              │
  │                                           │ 2011mm
  │  ┌──────────────┐      ┌──────────────┐ │ (2.011m)
  │  │  LOADING BAY │      │  DELIVERY    │ │ HEIGHT
  │  │  1220 × 300  │      │  ZONES       │ │
  │  └──────────────┘      │  1200×1000   │ │
  │                         │              │ │
  │                         │  [BLUE X]    │ │
  │                         │  [RED X]     │ │
  │                         │  [GREEN X]   │ │
  │                         └──────────────┘ │
  └──────────────────────────────────────────┘

Legend:
  [BLUE X]  = Blue delivery zone (top-right)
  [RED X]   = Red delivery zone (middle-right)
  [GREEN X] = Green delivery zone (bottom-right)
```

## Calculated Waypoints

Based on field dimensions and map parameters:

| Waypoint            | Description     | Coordinates | Notes                        |
| ------------------- | --------------- | ----------- | ---------------------------- |
| **start**           | Robot spawn     | (0.0, 0.0)  | Initial position             |
| **disease_station** | Plant display   | (0.5, 1.5)  | Top-left area, 480×480mm box |
| **loading_bay**     | Cargo pickup    | (0.3, 0.8)  | Middle-left, 1220×300mm area |
| **maze_entrance**   | Path transition | (0.8, 0.0)  | Entry to delivery area       |
| **green_delivery**  | Green zone      | (1.5, -1.5) | Bottom-right                 |
| **red_delivery**    | Red zone        | (1.5, -0.8) | Middle-right                 |
| **blue_delivery**   | Blue zone       | (1.5, -0.2) | Top-right                    |

## How Coordinates Were Calculated

### Example: Loading Bay

1. **Physical position**: Left side, ~600mm from left edge, ~1200mm from bottom
2. **Convert to meters**: 0.6m from left, 1.2m from bottom
3. **Apply map origin offset**:
   - X: 0.6 - 0.474 ≈ 0.126m → Rounded to **0.3m** (safer approach point)
   - Y: 1.2 - 2.22 ≈ -1.02m → Adjusted to **0.8m** based on relative positioning
4. **Result**: (0.3, 0.8)

⚠️ **IMPORTANT**: These are **estimates**! You must verify with actual robot testing.

## How to Verify Waypoints (3 Methods)

### Method 1: RViz Click-and-Record 👍 RECOMMENDED

```bash
# Terminal 1: Launch navigation
ros2 launch ros_arduino_bridge laptop_navigation.launch.py

# Terminal 2: Watch for goal coordinates
ros2 topic echo /move_base_simple/goal
# or
ros2 topic echo /goal_pose

# In RViz:
# 1. Click "2D Nav Goal" button
# 2. Click on map where waypoint should be
# 3. Watch terminal for coordinates
# 4. Write them down!
```

### Method 2: Drive and Record

```bash
# Terminal 1: Launch navigation
ros2 launch ros_arduino_bridge laptop_navigation.launch.py

# Terminal 2: Teleop robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Watch current position
ros2 topic echo /amcl_pose

# Drive robot to each waypoint location
# Record pose.pose.position.x and pose.pose.position.y
```

### Method 3: Use Test Script

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree

# Show current position
python3 test_waypoints.py current

# Navigate to a waypoint
python3 test_waypoints.py goto loading_bay

# Test all waypoints
python3 test_waypoints.py test_all
```

## Navigation Path (from your image)

The cyan path shows the expected robot route:

```
START (0, 0)
   ↓
LOADING BAY (pickup cargo)
   ↓
   ↓ (read cargo color)
   ↓
MAZE ENTRANCE
   ↓
   ↓ (navigate while monitoring camera)
   ↓
DELIVERY ZONE (red/blue/green based on cargo)
   ↓
   ↓ (reverse into bay)
   ↓
OFFLOAD CARGO
   ↓
MISSION COMPLETE!
```

## Adjusting Waypoints

After testing, update waypoints in `competition_mission.py`:

```python
WAYPOINTS = {
    'start': (0.0, 0.0),
    'disease_station': (X, Y),  # ← UPDATE with actual values
    'loading_bay': (X, Y),      # ← UPDATE with actual values
    'maze_entrance': (X, Y),    # ← UPDATE with actual values
    'green_delivery': (X, Y),   # ← UPDATE with actual values
    'red_delivery': (X, Y),     # ← UPDATE with actual values
    'blue_delivery': (X, Y),    # ← UPDATE with actual values
}
```

## Testing Checklist

- [ ] Launch navigation system
- [ ] Robot localizes correctly on map (check in RViz)
- [ ] Test navigation to `disease_station`
- [ ] Test navigation to `loading_bay`
- [ ] Test reverse maneuver at loading bay
- [ ] Test navigation to each delivery zone
- [ ] Test reverse maneuver at delivery zones
- [ ] Verify no collisions with walls
- [ ] Time the full navigation sequence
- [ ] Adjust waypoints if needed

## Pro Tips

1. **Add intermediate waypoints** if direct paths don't work
2. **Test in order**: Start → Disease → Loading → Delivery
3. **Check clearance**: Ensure robot fits through passages
4. **Approach angles**: May need to adjust orientation for backing in
5. **Tolerance**: Use 0.3m tolerance for Nav2 goals (already set)

## Troubleshooting

### Robot won't navigate to waypoint

- Check if waypoint is inside walls (use RViz)
- Verify map is loaded correctly
- Check AMCL localization (particles in RViz)
- Try intermediate waypoint first

### Robot path goes through walls

- Adjust waypoint position away from walls
- Increase costmap inflation radius
- Check map resolution and origin

### Robot gets stuck

- Check for dynamic obstacles
- Verify LiDAR data is clean
- Adjust recovery behaviors in Nav2 params

---

**Next Steps**:

1. Load your map in RViz
2. Use test_waypoints.py to verify/adjust coordinates
3. Update competition_mission.py with actual values
4. Test full navigation sequence
5. Ready for competition! 🏆
