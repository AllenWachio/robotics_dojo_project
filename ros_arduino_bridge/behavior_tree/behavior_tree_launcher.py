#!/usr/bin/env python3
"""
Master Behavior Tree Launcher (non-invasive)

This launcher composes the existing behavior-tree modules in this
directory into a single master tree so you can launch them together
without modifying the original files.

Usage examples:
  # Run only cube delivery mission
  python3 behavior_tree_launcher.py --subtrees cube

  # Run navigation demo + cube mission
  python3 behavior_tree_launcher.py --subtrees nav cube

  # Run everything (nav, cube, incremental)
  python3 behavior_tree_launcher.py --subtrees all

Notes:
- This file imports and composes existing behavior objects and factory
  functions. It does not modify any other files.
- If you want a different composition (Parallel vs Sequence), change the
  `MASTER_MODE` variable or pass an argument in the future.
"""

import argparse
import time
import os
import re

# Guard ROS message import so dry-run works without ROS installed
try:
    from geometry_msgs.msg import PoseWithCovarianceStamped
except Exception:
    PoseWithCovarianceStamped = None

# Import existing modules (non-invasive)
try:
    from cube_delivery_mission import create_cube_delivery_tree
except Exception:
    create_cube_delivery_tree = None

try:
    from robot_navigation_bt import create_root as create_navigation_tree
except Exception:
    create_navigation_tree = None

try:
    from incremental_move_behavior import IncrementalMove
except Exception:
    IncrementalMove = None


def build_master_tree(selected_subtrees, master_mode="sequence"):
    """
    Build a master py_trees tree by composing available subtree roots.

    Args:
        selected_subtrees: list of subtree keys to include (e.g. ['cube','nav'])
        master_mode: 'sequence' or 'parallel'

    Returns:
        py_trees.behaviour.Behaviour: root of the composed tree
    """
    if master_mode == "parallel":
        root = py_trees.composites.Parallel("MASTER_PARALLEL", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    else:
        root = py_trees.composites.Sequence("MASTER_SEQUENCE", memory=True)

    children = []

    if ("cube" in selected_subtrees or "all" in selected_subtrees) and create_cube_delivery_tree:
        try:
            cube_root = create_cube_delivery_tree()
            cube_root.name = "CubeDeliveryRoot"
            children.append(cube_root)
        except Exception as e:
            print(f"Warning: Failed to create cube delivery tree: {e}")

    if ("nav" in selected_subtrees or "all" in selected_subtrees) and create_navigation_tree:
        try:
            nav_root = create_navigation_tree()
            nav_root.name = "NavigationDemoRoot"
            children.append(nav_root)
        except Exception as e:
            print(f"Warning: Failed to create navigation tree: {e}")

    # Example: include a single IncrementalMove behavior as a small task
    if ("incremental" in selected_subtrees or "all" in selected_subtrees) and IncrementalMove:
        try:
            # Example coordinates; user can edit invocation later
            inc_move = IncrementalMove("Incremental_Move_Example", goal_x=1.0, goal_y=0.0)
            children.append(inc_move)
        except Exception as e:
            print(f"Warning: Failed to instantiate IncrementalMove: {e}")

    if not children:
        # Fall back to a simple noop behavior
        noop = py_trees.behaviours.Success("NoSubtrees_Selected")
        root.add_child(noop)
        return root

    # Add all child roots to master
    root.add_children(children)
    return root


def publish_initial_pose(node):
    """Publish a neutral initial pose to AMCL (map origin) to keep behavior examples consistent."""
    if PoseWithCovarianceStamped is None:
        print("Warning: ROS message types not available (dry-run environment). Skipping initial pose publish.")
        return

    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    time.sleep(0.5)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    try:
        msg.header.stamp = node.get_clock().now().to_msg()
    except Exception:
        pass
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    for _ in range(3):
        pub.publish(msg)
        time.sleep(0.05)
    print("Initial pose published to /initialpose")


def main():
    parser = argparse.ArgumentParser(description="Compose and launch multiple py_trees trees from this folder.")
    parser.add_argument('--subtrees', nargs='+', default=['cube'],
                        help="Which subtrees to include. Options: cube, nav, incremental, all")
    parser.add_argument('--mode', choices=['sequence', 'parallel'], default='sequence',
                        help='Master composition mode')
    parser.add_argument('--dry-run', action='store_true', help='Build and report tree composition without starting ROS/py_trees')
    args = parser.parse_args()

    selected = [s.lower() for s in args.subtrees]

    print(f"Selected subtrees: {selected}")

    # Dry-run mode: inspect files and report composition without importing ROS/py_trees
    if args.dry_run:
        dry_run_report(selected, args.mode)
        return

    # Normal run: import ROS and py_trees now (may raise if environment not configured)
    import rclpy
    import py_trees
    import py_trees_ros

    master_root = build_master_tree(selected, master_mode=args.mode)
    tree = py_trees_ros.trees.BehaviourTree(master_root)

    try:
        # Set up tree and pass the ROS node to behaviors that implement setup(node=...)
        tree.setup(timeout=10.0, node=tree.node)

        # Publish a neutral initial pose as examples rely on AMCL
        publish_initial_pose(tree.node)

        print("Starting master behavior tree...")

        tree_completed = False

        def tick_tree():
            nonlocal tree_completed
            if tree_completed:
                return
            tree.tick_tock(period_ms=500)
            if tree.root.status == py_trees.common.Status.SUCCESS:
                print("MASTER: SUCCESS")
                tree_completed = True
            elif tree.root.status == py_trees.common.Status.FAILURE:
                print("MASTER: FAILURE")
                tree_completed = True

        timer = tree.node.create_timer(0.5, tick_tree)
        rclpy.spin(tree.node)

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


def dry_run_report(selected_subtrees, mode):
    """Produce a safe dry-run report by scanning source files for factories/classes.

    This avoids importing py_trees or ROS. It reports which modules expose
    usable entry points and what would be composed into the master tree.
    """
    base = os.path.dirname(os.path.abspath(__file__))
    files = {
        'cube': os.path.join(base, 'cube_delivery_mission.py'),
        'nav': os.path.join(base, 'robot_navigation_bt.py'),
        'incremental': os.path.join(base, 'incremental_move_behavior.py'),
        'sensors': os.path.join(base, 'sensor_behaviors.py'),
        'inspector': os.path.join(base, 'inspect_timeout_implementation.py'),
        'waypoint': os.path.join(base, 'waypoint_converter_interactive.py'),
    }

    print('\n' + '='*70)
    print('DRY-RUN REPORT: Master Behavior Tree Composition')
    print('='*70)
    print(f"Mode: {mode}")
    print(f"Requested subtrees: {selected_subtrees}\n")

    for key in selected_subtrees:
        k = key.lower()
        if k == 'all':
            # report all known
            for name in ['cube', 'nav', 'incremental']:
                report_module(files, name)
            break
        report_module(files, k)

    print('\nSummary:')
    print(' - No ROS/py_trees imports were performed in dry-run.')
    print(' - To actually run the composed tree, rerun without --dry-run in a ROS2 environment with py_trees installed.')
    print('='*70 + '\n')


def report_module(files, key):
    path = files.get(key)
    if not path or not os.path.exists(path):
        print(f"[MISSING] {key}: file not found: {path}")
        return

    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check common entry points
    if key == 'cube':
        found = bool(re.search(r'def\s+create_cube_delivery_tree\s*\(', content))
        print(f"[MODULE] cube_delivery_mission.py -> create_cube_delivery_tree: {'FOUND' if found else 'NOT FOUND'})")
        if found:
            # try to summarize high-level phases from docstring
            m = re.search(r'"""(.*?)"""', content, re.DOTALL)
            if m:
                summary = m.group(1).strip().splitlines()[0:4]
                print('   Summary docstring (first lines):')
                for line in summary:
                    print('     ' + line.strip())
    elif key == 'nav':
        found = bool(re.search(r'def\s+create_root\s*\(', content)) or 'MoveToPosition' in content
        print(f"[MODULE] robot_navigation_bt.py -> create_root / MoveToPosition: {'FOUND' if found else 'NOT FOUND'})")
    elif key == 'incremental':
        found = bool(re.search(r'class\s+IncrementalMove\b', content))
        print(f"[MODULE] incremental_move_behavior.py -> class IncrementalMove: {'FOUND' if found else 'NOT FOUND'})")
    elif key == 'sensors':
        classes = ['WaitForDiseaseDetection', 'ReadColorSensor', 'MonitorCameraForColor']
        present = [c for c in classes if c in content]
        print(f"[MODULE] sensor_behaviors.py -> Behaviours found: {', '.join(present) if present else 'NONE'})")
    else:
        print(f"[MODULE] {os.path.basename(path)}: present (no specific factory checks)")
