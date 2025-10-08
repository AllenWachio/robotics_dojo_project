# Waypoint Calculation Guide

## Map Parameters

- **Resolution**: 0.05 m/pixel (5 cm/pixel)
- **Origin**: (-0.474, -2.22, 0) meters
- **Map Size**: 2400mm × 2011mm (2.4m × 2.011m)

## Field Dimensions (from image)

- Total width: 2400mm (2.4m)
- Total height: ~2011mm (2.011m)
- Plant display sections: 480mm × 480mm each (top)
- Loading bay: 1220mm × 300mm (middle left)
- Delivery zones: 1200mm × 1000mm (right side)

## Coordinate System

Map coordinates use the origin at bottom-left: (-0.474, -2.22)
To convert from pixel coordinates to map coordinates:

```
map_x = (pixel_x * resolution) + origin_x
map_y = (pixel_y * resolution) + origin_y
```

## Waypoint Calculations

### Starting Point

- **Location**: Bottom-left corner (assumed robot start)
- **Pixel coordinates**: (0, 0) approximately
- **Map coordinates**: (-0.474, -2.22)
- **Practical start**: (0.0, 0.0) - we can set initial pose here

### Disease Detection Station (Plant Display)

- **Location**: Top section, one of the 480×480 plant boxes
- **Physical**: ~600mm from left, ~1800mm from bottom
- **Pixel approx**: (12 pixels from left, 36 pixels from bottom)
- **Map coordinates**: (0.6 - 0.474, 1.8 - 2.22) = **(0.126, -0.42)**
- **Better estimate**: (0.5, 1.5) relative to start

### Loading Bay

- **Location**: Middle-left, 1220mm × 300mm area
- **Physical**: Left side, ~1200mm from bottom, 300mm tall
- **Center of loading bay**: ~600mm from left, ~1200mm from bottom
- **Map coordinates**: (0.6, 1.2) relative to origin shift
- **Practical**: **(0.3, 0.8)** for approach point

### Delivery Zones (from path image)

Based on the colored X markers in the navigation path:

1. **Green/Bottom Delivery Zone** (Green X)

   - **Location**: Bottom-right area
   - **Physical**: ~1800mm from left, ~400mm from bottom
   - **Map coordinates**: **(1.5, -1.5)**

2. **Red/Middle Delivery Zone** (Red X)

   - **Location**: Middle-right area
   - **Physical**: ~1800mm from left, ~1000mm from bottom
   - **Map coordinates**: **(1.5, -0.8)**

3. **Blue/Top Delivery Zone** (Blue X)
   - **Location**: Top-right corner of right section
   - **Physical**: ~1800mm from left, ~1600mm from bottom
   - **Map coordinates**: **(1.5, -0.2)**

### Maze Entrance

- **Location**: Transition from left side to right delivery area
- **Physical**: ~1200mm from left, ~1200mm from bottom
- **Map coordinates**: **(0.8, 0.0)**

## Recommended Waypoints for Competition

```python
WAYPOINTS = {
    # Starting position
    'start': (0.0, 0.0),

    # Disease detection (plant display area - top left)
    'disease_station': (0.5, 1.5),

    # Loading bay (middle left area)
    'loading_bay': (0.3, 0.8),

    # Maze entrance (transition to delivery area)
    'maze_entrance': (0.8, 0.0),

    # Delivery zones (right side)
    'green_delivery': (1.5, -1.5),   # Bottom right
    'red_delivery': (1.5, -0.8),     # Middle right
    'blue_delivery': (1.5, -0.2),    # Top right
}
```

## Notes

1. These coordinates are **estimates** based on image analysis
2. You should **measure actual positions** on your competition field
3. Use RViz to click positions and get exact coordinates
4. The path shows the robot navigates in a loop pattern
5. Adjust these based on your robot's actual testing

## How to Get Exact Coordinates

### Method 1: Use RViz (Recommended)

1. Launch your navigation system
2. Open RViz with your map
3. Use "2D Nav Goal" tool to click waypoints
4. Terminal will show goal coordinates - write them down!

### Method 2: Drive Robot Manually

1. Teleop robot to each waypoint location
2. Check `/amcl_pose` topic for coordinates:
   ```bash
   ros2 topic echo /amcl_pose
   ```
3. Record x and y values

### Method 3: Map Pixel Measurement

1. Open gamefield.pgm in image editor
2. Measure pixel coordinates of waypoints
3. Convert using:
   ```
   map_x = (pixel_x * 0.05) - 0.474
   map_y = (pixel_y * 0.05) - 2.22
   ```
