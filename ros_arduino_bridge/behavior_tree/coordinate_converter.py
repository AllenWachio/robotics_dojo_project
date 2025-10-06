#!/usr/bin/env python3
"""
Coordinate Conversion Tool for Robotics Dojo Game Field
Converts between:
1. Real-world dimensions (mm)
2. Map image pixels
3. ROS map coordinates (meters)
"""

import yaml
import math
from dataclasses import dataclass
from typing import Tuple, List
import json


@dataclass
class MapConfig:
    """Map configuration from .yaml file"""
    image_width: int  # pixels
    image_height: int  # pixels
    resolution: float  # meters per pixel
    origin_x: float  # meters
    origin_y: float  # meters
    origin_yaw: float = 0.0  # radians


@dataclass
class GameFieldConfig:
    """Real-world game field dimensions"""
    width_mm: float = 2400.0  # millimeters
    height_mm: float = 2011.0  # millimeters
    
    @property
    def width_m(self):
        return self.width_mm / 1000.0
    
    @property
    def height_m(self):
        return self.height_mm / 1000.0


class CoordinateConverter:
    """
    Handles coordinate conversions between different reference frames
    """
    
    def __init__(self, map_yaml_path: str):
        """
        Initialize converter with map configuration
        
        Args:
            map_yaml_path: Path to the map .yaml file
        """
        self.map_config = self.load_map_config(map_yaml_path)
        self.field_config = GameFieldConfig()
        
    def load_map_config(self, yaml_path: str) -> MapConfig:
        """Load map configuration from YAML file"""
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Read PGM file to get dimensions
        pgm_path = yaml_path.replace('.yaml', '.pgm')
        width, height = self.read_pgm_dimensions(pgm_path)
        
        origin = config.get('origin', [0, 0, 0])
        
        return MapConfig(
            image_width=width,
            image_height=height,
            resolution=config['resolution'],
            origin_x=origin[0],
            origin_y=origin[1],
            origin_yaw=origin[2] if len(origin) > 2 else 0.0
        )
    
    def read_pgm_dimensions(self, pgm_path: str) -> Tuple[int, int]:
        """Read PGM image dimensions from header"""
        with open(pgm_path, 'rb') as f:
            # Skip magic number
            f.readline()
            # Skip comments
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            # Read dimensions
            width, height = map(int, line.split())
        return width, height
    
    # ============================================================
    # REAL WORLD (mm) → ROS MAP (meters)
    # ============================================================
    
    def realworld_to_ros(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """
        Convert real-world coordinates (mm) to ROS map coordinates (meters)
        
        Args:
            x_mm: X coordinate in millimeters (game field coordinates)
            y_mm: Y coordinate in millimeters (game field coordinates)
            
        Returns:
            (x_ros, y_ros): Coordinates in ROS map frame (meters)
        """
        # Convert mm to meters
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        
        # Add map origin offset
        x_ros = x_m + self.map_config.origin_x
        y_ros = y_m + self.map_config.origin_y
        
        return x_ros, y_ros
    
    def ros_to_realworld(self, x_ros: float, y_ros: float) -> Tuple[float, float]:
        """
        Convert ROS map coordinates (meters) to real-world coordinates (mm)
        
        Args:
            x_ros: X coordinate in ROS map frame (meters)
            y_ros: Y coordinate in ROS map frame (meters)
            
        Returns:
            (x_mm, y_mm): Coordinates in millimeters (game field)
        """
        # Remove map origin offset
        x_m = x_ros - self.map_config.origin_x
        y_m = y_ros - self.map_config.origin_y
        
        # Convert meters to mm
        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        
        return x_mm, y_mm
    
    # ============================================================
    # MAP PIXELS → ROS MAP (meters)
    # ============================================================
    
    def pixel_to_ros(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to ROS map coordinates
        
        Args:
            px: X pixel coordinate (0 = left edge)
            py: Y pixel coordinate (0 = top edge)
            
        Returns:
            (x_ros, y_ros): Coordinates in ROS map frame (meters)
        """
        # In images, Y increases downward, but in ROS it increases upward
        # So we need to flip Y
        py_flipped = self.map_config.image_height - py
        
        # Convert pixels to meters
        x_m = px * self.map_config.resolution
        y_m = py_flipped * self.map_config.resolution
        
        # Add origin offset
        x_ros = x_m + self.map_config.origin_x
        y_ros = y_m + self.map_config.origin_y
        
        return x_ros, y_ros
    
    def ros_to_pixel(self, x_ros: float, y_ros: float) -> Tuple[int, int]:
        """
        Convert ROS map coordinates to pixel coordinates
        
        Args:
            x_ros: X coordinate in ROS map frame (meters)
            y_ros: Y coordinate in ROS map frame (meters)
            
        Returns:
            (px, py): Pixel coordinates in image
        """
        # Remove origin offset
        x_m = x_ros - self.map_config.origin_x
        y_m = y_ros - self.map_config.origin_y
        
        # Convert meters to pixels
        px = int(x_m / self.map_config.resolution)
        py_flipped = int(y_m / self.map_config.resolution)
        
        # Flip Y back to image coordinates
        py = self.map_config.image_height - py_flipped
        
        return px, py
    
    # ============================================================
    # REAL WORLD (mm) → MAP PIXELS
    # ============================================================
    
    def realworld_to_pixel(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """
        Convert real-world coordinates (mm) to pixel coordinates
        
        Args:
            x_mm: X coordinate in millimeters
            y_mm: Y coordinate in millimeters
            
        Returns:
            (px, py): Pixel coordinates in image
        """
        # First convert to ROS, then to pixels
        x_ros, y_ros = self.realworld_to_ros(x_mm, y_mm)
        return self.ros_to_pixel(x_ros, y_ros)
    
    def pixel_to_realworld(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to real-world coordinates (mm)
        
        Args:
            px: X pixel coordinate
            py: Y pixel coordinate
            
        Returns:
            (x_mm, y_mm): Coordinates in millimeters
        """
        # First convert to ROS, then to real-world
        x_ros, y_ros = self.pixel_to_ros(px, py)
        return self.ros_to_realworld(x_ros, y_ros)
    
    # ============================================================
    # WAYPOINT GENERATION
    # ============================================================
    
    def generate_waypoints_mm(self, start_mm: Tuple[float, float], 
                              end_mm: Tuple[float, float], 
                              step_size_mm: float = 200.0) -> List[Tuple[float, float]]:
        """
        Generate intermediate waypoints in real-world coordinates (mm)
        
        Args:
            start_mm: Starting position (x_mm, y_mm)
            end_mm: Ending position (x_mm, y_mm)
            step_size_mm: Distance between waypoints in millimeters
            
        Returns:
            List of waypoints [(x_mm, y_mm), ...]
        """
        x1, y1 = start_mm
        x2, y2 = end_mm
        
        # Calculate total distance
        dx = x2 - x1
        dy = y2 - y1
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate number of steps
        num_steps = max(1, int(total_distance / step_size_mm))
        
        # Generate waypoints
        waypoints = []
        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            wp_x = x1 + dx * ratio
            wp_y = y1 + dy * ratio
            waypoints.append((wp_x, wp_y))
        
        return waypoints
    
    def generate_waypoints_ros(self, start_ros: Tuple[float, float], 
                               end_ros: Tuple[float, float], 
                               step_size_m: float = 0.2) -> List[Tuple[float, float]]:
        """
        Generate intermediate waypoints in ROS map coordinates (meters)
        
        Args:
            start_ros: Starting position (x_m, y_m)
            end_ros: Ending position (x_m, y_m)
            step_size_m: Distance between waypoints in meters
            
        Returns:
            List of waypoints [(x_m, y_m), ...]
        """
        x1, y1 = start_ros
        x2, y2 = end_ros
        
        # Calculate total distance
        dx = x2 - x1
        dy = y2 - y1
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate number of steps
        num_steps = max(1, int(total_distance / step_size_m))
        
        # Generate waypoints
        waypoints = []
        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            wp_x = x1 + dx * ratio
            wp_y = y1 + dy * ratio
            waypoints.append((wp_x, wp_y))
        
        return waypoints
    
    # ============================================================
    # INFORMATION & DEBUGGING
    # ============================================================
    
    def print_map_info(self):
        """Print map configuration information"""
        print("=" * 60)
        print("MAP CONFIGURATION")
        print("=" * 60)
        print(f"Image size:       {self.map_config.image_width} × {self.map_config.image_height} pixels")
        print(f"Resolution:       {self.map_config.resolution} m/pixel ({self.map_config.resolution*1000:.1f} mm/pixel)")
        print(f"Map dimensions:   {self.map_config.image_width * self.map_config.resolution:.3f} × {self.map_config.image_height * self.map_config.resolution:.3f} meters")
        print(f"Map origin:       ({self.map_config.origin_x:.3f}, {self.map_config.origin_y:.3f}) meters")
        print()
        print("GAME FIELD")
        print("=" * 60)
        print(f"Field size:       {self.field_config.width_mm} × {self.field_config.height_mm} mm")
        print(f"Field size:       {self.field_config.width_m:.3f} × {self.field_config.height_m:.3f} meters")
        print("=" * 60)
    
    def export_waypoints_csv(self, waypoints_ros: List[Tuple[float, float]], 
                            filename: str = "waypoints.csv"):
        """
        Export waypoints to CSV file
        
        Args:
            waypoints_ros: List of waypoints in ROS coordinates [(x, y), ...]
            filename: Output filename
        """
        import csv
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'x_ros_m', 'y_ros_m', 'x_mm', 'y_mm', 'pixel_x', 'pixel_y'])
            
            for idx, (x_ros, y_ros) in enumerate(waypoints_ros):
                x_mm, y_mm = self.ros_to_realworld(x_ros, y_ros)
                px, py = self.ros_to_pixel(x_ros, y_ros)
                writer.writerow([idx, f"{x_ros:.3f}", f"{y_ros:.3f}", 
                               f"{x_mm:.1f}", f"{y_mm:.1f}", px, py])
        
        print(f"Waypoints exported to {filename}")


# ============================================================
# EXAMPLE USAGE
# ============================================================

def main():
    """Example usage of coordinate converter"""
    import sys
    import os
    
    # Get map file path
    map_yaml = os.path.expanduser("~/ros2_ws/maps/gamefield.yaml")
    
    if not os.path.exists(map_yaml):
        print(f"Error: Map file not found: {map_yaml}")
        return
    
    # Create converter
    converter = CoordinateConverter(map_yaml)
    
    # Print map information
    converter.print_map_info()
    
    print("\n" + "=" * 60)
    print("EXAMPLE CONVERSIONS")
    print("=" * 60)
    
    # Example 1: Origin point (Green X in your image)
    print("\n1. Origin Point (Green X) - assumed at field corner:")
    x_mm, y_mm = 0.0, 0.0
    x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
    px, py = converter.realworld_to_pixel(x_mm, y_mm)
    print(f"   Real-world: ({x_mm:.0f}, {y_mm:.0f}) mm")
    print(f"   ROS coords: ({x_ros:.3f}, {y_ros:.3f}) m")
    print(f"   Pixel:      ({px}, {py})")
    
    # Example 2: Middle of field
    print("\n2. Center of Field:")
    x_mm, y_mm = 1200.0, 1005.5  # Half of 2400 and 2011
    x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
    px, py = converter.realworld_to_pixel(x_mm, y_mm)
    print(f"   Real-world: ({x_mm:.0f}, {y_mm:.0f}) mm")
    print(f"   ROS coords: ({x_ros:.3f}, {y_ros:.3f}) m")
    print(f"   Pixel:      ({px}, {py})")
    
    # Example 3: Top-right corner
    print("\n3. Top-Right Corner of Field:")
    x_mm, y_mm = 2400.0, 2011.0
    x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
    px, py = converter.realworld_to_pixel(x_mm, y_mm)
    print(f"   Real-world: ({x_mm:.0f}, {y_mm:.0f}) mm")
    print(f"   ROS coords: ({x_ros:.3f}, {y_ros:.3f}) m")
    print(f"   Pixel:      ({px}, {py})")
    
    # Example 4: Generate waypoints
    print("\n4. Generate Waypoints (0,0) to (2400, 2011) with 500mm steps:")
    waypoints_mm = converter.generate_waypoints_mm((0, 0), (2400, 2011), step_size_mm=500.0)
    print(f"   Generated {len(waypoints_mm)} waypoints:")
    for i, (x, y) in enumerate(waypoints_mm[:5]):  # Show first 5
        x_ros, y_ros = converter.realworld_to_ros(x, y)
        print(f"   WP{i+1}: ({x:.0f}, {y:.0f}) mm → ({x_ros:.3f}, {y_ros:.3f}) m")
    if len(waypoints_mm) > 5:
        print(f"   ... and {len(waypoints_mm) - 5} more")
    
    # Example 5: Reverse conversion (ROS to real-world)
    print("\n5. Reverse Conversion (ROS → Real-world):")
    x_ros, y_ros = 0.5, 0.5  # Some ROS coordinate
    x_mm, y_mm = converter.ros_to_realworld(x_ros, y_ros)
    print(f"   ROS coords: ({x_ros:.3f}, {y_ros:.3f}) m")
    print(f"   Real-world: ({x_mm:.0f}, {y_mm:.0f}) mm")
    
    print("\n" + "=" * 60)
    print("✅ Conversion tool ready to use!")
    print("=" * 60)


if __name__ == "__main__":
    main()
