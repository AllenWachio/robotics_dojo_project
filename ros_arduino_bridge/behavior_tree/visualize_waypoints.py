#!/usr/bin/env python3
"""
Waypoint Visualization Publisher
=================================
Publishes markers to visualize all waypoints in RViz

Usage:
    ros2 run ros_arduino_bridge visualize_waypoints.py
    # Or directly:
    python3 visualize_waypoints.py

Then in RViz:
    Add -> MarkerArray -> Topic: /waypoint_markers

Author: Robotics Dojo 2025
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Import waypoints
from competition_mission import WAYPOINTS


class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        # Timer to publish markers
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Waypoint Visualizer started')
        self.get_logger().info('Add MarkerArray display in RViz:')
        self.get_logger().info('  Topic: /waypoint_markers')
    
    def publish_markers(self):
        """Publish visualization markers for all waypoints"""
        marker_array = MarkerArray()
        
        # Color scheme
        colors = {
            'start': ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),          # White
            'disease_station': ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0), # Orange
            'loading_bay': ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),    # Yellow
            'maze_entrance': ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0),  # Gray
            'green_delivery': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), # Green
            'red_delivery': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),   # Red
            'blue_delivery': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Blue
        }
        
        marker_id = 0
        
        for name, (x, y) in WAYPOINTS.items():
            # Sphere marker for waypoint
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.2  # Slightly above ground
            
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color = colors.get(name, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # Text marker for waypoint name
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_names'
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = 0.5  # Above sphere
            
            text_marker.text = name
            text_marker.scale.z = 0.2  # Text size
            
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            marker_array.markers.append(text_marker)
            marker_id += 1
        
        # Add arrows showing expected path
        path_segments = [
            ('start', 'disease_station'),
            ('disease_station', 'start'),
            ('start', 'loading_bay'),
            ('loading_bay', 'maze_entrance'),
            ('maze_entrance', 'red_delivery'),  # Example path
        ]
        
        for start_name, end_name in path_segments:
            if start_name in WAYPOINTS and end_name in WAYPOINTS:
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'path'
                arrow.id = marker_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                start_x, start_y = WAYPOINTS[start_name]
                end_x, end_y = WAYPOINTS[end_name]
                
                # Start point
                start_point = Point()
                start_point.x = float(start_x)
                start_point.y = float(start_y)
                start_point.z = 0.1
                
                # End point
                end_point = Point()
                end_point.x = float(end_x)
                end_point.y = float(end_y)
                end_point.z = 0.1
                
                arrow.points = [start_point, end_point]
                
                arrow.scale.x = 0.05  # Shaft diameter
                arrow.scale.y = 0.1   # Head diameter
                arrow.scale.z = 0.1   # Head length
                
                arrow.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)  # Cyan, semi-transparent
                
                marker_array.markers.append(arrow)
                marker_id += 1
        
        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    visualizer = WaypointVisualizer()
    
    try:
        print("\n" + "="*60)
        print("WAYPOINT VISUALIZER")
        print("="*60)
        print("\nPublishing waypoint markers to /waypoint_markers")
        print("\nTo view in RViz:")
        print("  1. Open RViz")
        print("  2. Click 'Add' button")
        print("  3. Select 'MarkerArray'")
        print("  4. Set Topic to '/waypoint_markers'")
        print("\nWaypoint colors:")
        print("  White  = Start")
        print("  Orange = Disease Station")
        print("  Yellow = Loading Bay")
        print("  Gray   = Maze Entrance")
        print("  Green  = Green Delivery")
        print("  Red    = Red Delivery")
        print("  Blue   = Blue Delivery")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")
        
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
