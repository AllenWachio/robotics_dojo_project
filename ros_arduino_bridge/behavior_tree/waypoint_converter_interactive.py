#!/usr/bin/env python3
"""
Interactive Waypoint Converter for Game Field
Helps you convert measurements from your game field to ROS coordinates
"""

from coordinate_converter import CoordinateConverter
import os


def print_header():
    print("\n" + "=" * 70)
    print("🎯  GAME FIELD WAYPOINT CONVERTER")
    print("=" * 70)


def print_menu():
    print("\nWhat would you like to do?")
    print("  1. Convert single waypoint (mm → ROS meters)")
    print("  2. Convert multiple waypoints from list")
    print("  3. Generate intermediate waypoints")
    print("  4. Show map information")
    print("  5. Export waypoints to behavior tree format")
    print("  6. Exit")
    print()


def convert_single_waypoint(converter):
    """Convert a single waypoint interactively"""
    print("\n" + "-" * 70)
    print("CONVERT SINGLE WAYPOINT")
    print("-" * 70)
    
    try:
        x_mm = float(input("Enter X coordinate (mm): "))
        y_mm = float(input("Enter Y coordinate (mm): "))
        
        # Convert
        x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
        px, py = converter.realworld_to_pixel(x_mm, y_mm)
        
        # Display results
        print("\n✅ CONVERSION RESULTS:")
        print(f"   Real-world:  ({x_mm:.0f}, {y_mm:.0f}) mm")
        print(f"   ROS coords:  ({x_ros:.3f}, {y_ros:.3f}) meters")
        print(f"   Pixel:       ({px}, {py})")
        print(f"\n💡 Use in behavior tree:")
        print(f"   move = MoveToPosition('Waypoint', {x_ros:.3f}, {y_ros:.3f}, tolerance=0.2)")
        
    except ValueError:
        print("❌ Invalid input. Please enter numbers only.")


def convert_multiple_waypoints(converter):
    """Convert multiple waypoints from input"""
    print("\n" + "-" * 70)
    print("CONVERT MULTIPLE WAYPOINTS")
    print("-" * 70)
    print("Enter waypoints one per line as: X,Y (in mm)")
    print("Example: 1200,1000")
    print("Enter 'done' when finished")
    print()
    
    waypoints = []
    while True:
        line = input(f"Waypoint {len(waypoints)+1}: ").strip()
        
        if line.lower() == 'done':
            break
        
        try:
            x_mm, y_mm = map(float, line.split(','))
            waypoints.append((x_mm, y_mm))
            x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
            print(f"   → ROS: ({x_ros:.3f}, {y_ros:.3f}) m ✓")
        except ValueError:
            print("   ❌ Invalid format. Use: X,Y (e.g., 1200,1000)")
    
    if waypoints:
        print(f"\n✅ Converted {len(waypoints)} waypoints!")
        print("\n📊 SUMMARY:")
        print(f"{'#':<4} {'Real-world (mm)':<20} {'ROS (meters)':<20}")
        print("-" * 50)
        
        for i, (x_mm, y_mm) in enumerate(waypoints, 1):
            x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
            print(f"{i:<4} ({x_mm:.0f}, {y_mm:.0f}){'':<12} ({x_ros:.3f}, {y_ros:.3f})")
        
        return waypoints
    else:
        print("No waypoints entered.")
        return []


def generate_intermediate_waypoints(converter):
    """Generate waypoints between start and end"""
    print("\n" + "-" * 70)
    print("GENERATE INTERMEDIATE WAYPOINTS")
    print("-" * 70)
    
    try:
        # Get start point
        print("\nStart point (mm):")
        start_x = float(input("  X: "))
        start_y = float(input("  Y: "))
        
        # Get end point
        print("\nEnd point (mm):")
        end_x = float(input("  X: "))
        end_y = float(input("  Y: "))
        
        # Get step size
        step_size = float(input("\nStep size (mm, e.g., 200 for 20cm steps): "))
        
        # Generate waypoints
        waypoints_mm = converter.generate_waypoints_mm(
            (start_x, start_y), 
            (end_x, end_y), 
            step_size
        )
        
        print(f"\n✅ Generated {len(waypoints_mm)} intermediate waypoints!")
        print(f"\n📊 WAYPOINTS:")
        print(f"{'#':<4} {'Real-world (mm)':<20} {'ROS (meters)':<20}")
        print("-" * 50)
        
        # Show start
        x_ros, y_ros = converter.realworld_to_ros(start_x, start_y)
        print(f"{'S':<4} ({start_x:.0f}, {start_y:.0f}){'':<12} ({x_ros:.3f}, {y_ros:.3f}) ← Start")
        
        # Show waypoints
        for i, (x_mm, y_mm) in enumerate(waypoints_mm, 1):
            x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
            print(f"{i:<4} ({x_mm:.0f}, {y_mm:.0f}){'':<12} ({x_ros:.3f}, {y_ros:.3f})")
        
        return waypoints_mm
        
    except ValueError:
        print("❌ Invalid input. Please enter numbers only.")
        return []


def show_map_info(converter):
    """Display map configuration"""
    converter.print_map_info()


def export_behavior_tree_code(converter, waypoints_mm):
    """Export waypoints as Python behavior tree code"""
    if not waypoints_mm:
        print("\n❌ No waypoints to export. Please convert waypoints first.")
        return
    
    print("\n" + "-" * 70)
    print("BEHAVIOR TREE CODE")
    print("-" * 70)
    print("\n# Copy this code into your behavior tree:\n")
    
    print("def create_root():")
    print("    root = py_trees.composites.Sequence('GameFieldMission', memory=True)")
    print()
    
    for i, (x_mm, y_mm) in enumerate(waypoints_mm, 1):
        x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
        print(f"    # Waypoint {i}: ({x_mm:.0f}, {y_mm:.0f}) mm")
        print(f"    move{i} = MoveToPosition('WP{i}', {x_ros:.3f}, {y_ros:.3f}, tolerance=0.2)")
        if i < len(waypoints_mm):
            print()
    
    print()
    print("    # Add all movements to sequence")
    moves = ", ".join([f"move{i+1}" for i in range(len(waypoints_mm))])
    print(f"    root.add_children([{moves}])")
    print("    return root")
    print()
    
    # Save to file
    output_file = "generated_waypoints.py"
    with open(output_file, 'w') as f:
        f.write("# Auto-generated waypoints\n")
        f.write("# Generated from real-world coordinates\n\n")
        f.write("def create_root():\n")
        f.write("    root = py_trees.composites.Sequence('GameFieldMission', memory=True)\n\n")
        
        for i, (x_mm, y_mm) in enumerate(waypoints_mm, 1):
            x_ros, y_ros = converter.realworld_to_ros(x_mm, y_mm)
            f.write(f"    # Waypoint {i}: ({x_mm:.0f}, {y_mm:.0f}) mm\n")
            f.write(f"    move{i} = MoveToPosition('WP{i}', {x_ros:.3f}, {y_ros:.3f}, tolerance=0.2)\n")
            if i < len(waypoints_mm):
                f.write("\n")
        
        f.write("\n    # Add all movements to sequence\n")
        moves = ", ".join([f"move{i+1}" for i in range(len(waypoints_mm))])
        f.write(f"    root.add_children([{moves}])\n")
        f.write("    return root\n")
    
    print(f"✅ Code saved to: {output_file}")


def main():
    """Main interactive loop"""
    
    # Initialize converter
    map_yaml = os.path.expanduser("~/ros2_ws/maps/gamefield.yaml")
    
    if not os.path.exists(map_yaml):
        print(f"❌ Error: Map file not found: {map_yaml}")
        print("Please make sure your map is at ~/ros2_ws/maps/gamefield.yaml")
        return
    
    converter = CoordinateConverter(map_yaml)
    waypoints_mm = []  # Store for export
    
    print_header()
    converter.print_map_info()
    
    while True:
        print_menu()
        
        try:
            choice = input("Enter choice (1-6): ").strip()
            
            if choice == '1':
                convert_single_waypoint(converter)
            
            elif choice == '2':
                waypoints_mm = convert_multiple_waypoints(converter)
            
            elif choice == '3':
                waypoints_mm = generate_intermediate_waypoints(converter)
            
            elif choice == '4':
                show_map_info(converter)
            
            elif choice == '5':
                export_behavior_tree_code(converter, waypoints_mm)
            
            elif choice == '6':
                print("\n👋 Goodbye!")
                break
            
            else:
                print("❌ Invalid choice. Please enter 1-6.")
        
        except KeyboardInterrupt:
            print("\n\n👋 Interrupted. Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()
