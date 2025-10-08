#!/usr/bin/env python3
"""
Competition Mission Launch Script
==================================
Easy-to-use launcher for competition day

Usage:
    python3 launch_competition.py [--dry-run] [--no-disease]

Options:
    --dry-run      Show tree structure without executing
    --no-disease   Skip disease detection phase

Author: Robotics Dojo 2025
"""

import sys
import argparse
import subprocess
import time


def check_prerequisites():
    """Check if all required nodes are running"""
    print("\n🔍 Checking prerequisites...")
    
    required_nodes = [
        '/ros_arduino_bridge',
        '/color_detection_node',
        '/disease_detection_node',
    ]
    
    # Get list of active nodes
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        capture_output=True,
        text=True
    )
    
    active_nodes = result.stdout.strip().split('\n')
    
    all_ok = True
    for node in required_nodes:
        if node in active_nodes:
            print(f"  ✓ {node}")
        else:
            print(f"  ❌ {node} NOT RUNNING")
            all_ok = False
    
    return all_ok


def check_nav2():
    """Check if Nav2 is running"""
    print("\n🗺️  Checking Nav2...")
    
    result = subprocess.run(
        ['ros2', 'action', 'list'],
        capture_output=True,
        text=True
    )
    
    if '/navigate_to_pose' in result.stdout:
        print("  ✓ Nav2 navigation server active")
        return True
    else:
        print("  ❌ Nav2 NOT RUNNING")
        return False


def show_tree_structure():
    """Display behavior tree structure"""
    print("\n🌳 Competition Mission Tree Structure:")
    print("="*70)
    
    # Import and display tree
    from competition_mission import create_competition_tree
    import py_trees
    
    root = create_competition_tree()
    py_trees.display.print_ascii_tree(root, show_status=False)
    print("="*70)


def launch_mission(dry_run=False, disease_enabled=True):
    """Launch the competition mission"""
    
    if dry_run:
        show_tree_structure()
        return
    
    # Check prerequisites
    if not check_prerequisites():
        print("\n❌ Prerequisites not met!")
        print("\nMake sure these are running:")
        print("  1. ros2 launch ros_arduino_bridge full_slam_test.launch.py")
        print("  2. ros2 run rpi_camera_package color_detection_node")
        print("  3. ros2 run rdj2025_potato_disease_detection potato_disease_detection_node")
        return
    
    if not check_nav2():
        print("\n❌ Nav2 not running!")
        print("\nStart Nav2 with:")
        print("  ros2 launch ros_arduino_bridge laptop_navigation.launch.py")
        return
    
    print("\n✓ All prerequisites met!")
    print("\n" + "="*70)
    print("🚀 LAUNCHING COMPETITION MISSION")
    print("="*70)
    
    if not disease_enabled:
        print("⚠️  Disease detection DISABLED")
    
    print("\nStarting in 3 seconds...")
    time.sleep(3)
    
    # Build command
    cmd = ['python3', 'competition_mission.py']
    
    # Set ROS parameter for disease detection
    import os
    if not disease_enabled:
        os.environ['ROS_PARAM_DISEASE_DETECTION_ENABLED'] = 'false'
    
    # Run mission
    subprocess.run(cmd)


def main():
    """Parse arguments and launch"""
    parser = argparse.ArgumentParser(
        description='Launch autonomous robot competition mission'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show tree structure without executing'
    )
    parser.add_argument(
        '--no-disease',
        action='store_true',
        help='Skip disease detection phase'
    )
    
    args = parser.parse_args()
    
    launch_mission(
        dry_run=args.dry_run,
        disease_enabled=not args.no_disease
    )


if __name__ == "__main__":
    main()
