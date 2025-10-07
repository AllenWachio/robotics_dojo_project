#!/usr/bin/env python3
"""
Behavior Tree Orchestrator

This orchestrator provides a safe, non-invasive wrapper around the
existing behavior tree scripts in this folder. It does NOT modify any
other files. Use it to run dry-runs, print a preflight runbook checklist,
run unit/component tests, and (with explicit confirmation) start the
master mission launcher.

Usage examples:
  # Show help
  python3 behavior_tree_orchestrator.py --help

  # Print runbook / preflight checklist
  python3 behavior_tree_orchestrator.py runbook

  # Do a dry-run composition report (safe)
  python3 behavior_tree_orchestrator.py dry-run --subtrees all --mode sequence

  # Run the behavior_tree_launcher (requires ROS environment); --force required
  python3 behavior_tree_orchestrator.py run --subtrees cube nav --mode parallel --force

  # Run component tests (safe defaults)
  python3 behavior_tree_orchestrator.py test --which color

This file intentionally avoids importing ROS or py_trees at module import
time; imports are performed on-demand so the orchestrator can be used in
environments without ROS present.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from typing import List


def repo_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def print_runbook():
    """Print a short preflight runbook and checklist for safe execution."""
    print("\n=== Behavior Tree Orchestrator: Preflight Runbook ===\n")
    print("1) Prepare your ROS2 environment:")
    print("   - Source the ROS2 distro: source /opt/ros/humble/setup.bash")
    print("   - (Optional) source your workspace overlay: source ~/ros2_ws/install/setup.bash")
    print("\n2) Build and install dependencies: ")
    print("   - Ensure py_trees / py_trees_ros are available. Example:")
    print("       python3 -m pip install --user py_trees")
    print("       colcon build --packages-select py_trees_ros && source install/setup.bash")
    print("\n3) Start required middleware / systems before running missions:")
    print("   - Start navigation (AMCL, map_server, nav2) and any hardware nodes (camera, motors).\n")
    print("4) Run a dry-run first (safe):")
    print("   - python3 behavior_tree_orchestrator.py dry-run --subtrees all --mode sequence\n")
    print("5) If dry-run looks good, run tests: behavior_tree_orchestrator.py test --which all")
    print("6) When ready, start mission with --force (explicit):\n")
    print("   - python3 behavior_tree_orchestrator.py run --subtrees all --mode sequence --force\n")
    print("=== End runbook ===\n")


def do_dry_run(subtrees: List[str], mode: str):
    """Call the dry-run reporter in behavior_tree_launcher (if available),
    otherwise run the launcher script with --dry-run as a subprocess.
    """
    base = repo_dir()
    launcher_mod = None
    try:
        # Import on-demand; this keeps top-level import safe in non-ROS envs
        sys.path.insert(0, base)
        import behavior_tree_launcher as btl
        launcher_mod = btl
    except Exception:
        launcher_mod = None
    finally:
        try:
            sys.path.pop(0)
        except Exception:
            pass

    if launcher_mod and hasattr(launcher_mod, 'dry_run_report'):
        print("Using in-process dry-run reporter from behavior_tree_launcher\n")
        launcher_mod.dry_run_report(subtrees, mode)
        return 0

    # Fallback: run subprocess
    script = os.path.join(base, 'behavior_tree_launcher.py')
    cmd = [sys.executable, script, '--dry-run', '--mode', mode] + subtrees
    print(f"Falling back to running: {' '.join(cmd)}\n")
    return subprocess.run(cmd).returncode


def run_tests(which: str = 'all') -> int:
    """Run test scripts in this folder. This is intentionally conservative —
    tests that may actuate hardware should be avoided unless the user
    explicitly requests them with environment variables or flags.

    which: 'color', 'incremental', 'mission', or 'all'
    """
    base = repo_dir()
    tests = []
    if which in ('color', 'all'):
        # test_mission_components.py can run simple sensor checks
        tests.append(os.path.join(base, 'test_mission_components.py'))
    if which in ('incremental', 'all'):
        tests.append(os.path.join(base, 'test_incremental_navigation.py'))
    if which in ('mission', 'all'):
        tests.append(os.path.join(base, 'test_disease_timeout.py'))

    if not tests:
        print(f"No tests matched for '{which}'")
        return 2

    rc_total = 0
    for t in tests:
        if not os.path.exists(t):
            print(f"Skipping missing test: {t}")
            continue
        print(f"Running test: {t}")
        proc = subprocess.run([sys.executable, t])
        if proc.returncode != 0:
            print(f"Test {os.path.basename(t)} returned {proc.returncode}")
            rc_total = proc.returncode or rc_total

    return rc_total


def run_launcher(subtrees: List[str], mode: str, force: bool = False) -> int:
    """Start the real behavior tree launcher. This will import ROS and
    py_trees and may actuate hardware. Require --force to proceed.
    """
    if not force:
        print("Refusing to run mission: pass --force to confirm you understand this will start ROS/py_trees and may actuate hardware.")
        return 3

    base = repo_dir()
    script = os.path.join(base, 'behavior_tree_launcher.py')

    cmd = [sys.executable, script, '--mode', mode] + ['--subtrees'] + subtrees
    print(f"Starting launcher with: {' '.join(cmd)}")
    print("Note: make sure your ROS environment is sourced and required nodes (AMCL, nav2, camera) are running.")
    return subprocess.run(cmd).returncode


def parse_args(argv: List[str]):
    p = argparse.ArgumentParser(prog='behavior_tree_orchestrator', description='Orchestrate behavior tree dry-runs, tests and mission starts (non-invasive)')
    sub = p.add_subparsers(dest='cmd', required=True)

    # runbook
    sub.add_parser('runbook', help='Print runbook and preflight checklist')

    # dry-run
    dr = sub.add_parser('dry-run', help='Run a safe dry-run composition report')
    dr.add_argument('--subtrees', nargs='+', default=['cube'], help='Subtrees to include: cube, nav, incremental, all')
    dr.add_argument('--mode', choices=['sequence', 'parallel'], default='sequence')

    # test
    tt = sub.add_parser('test', help='Run component tests in this folder (conservative defaults)')
    tt.add_argument('--which', choices=['color', 'incremental', 'mission', 'all'], default='color')

    # run mission
    rn = sub.add_parser('run', help='Start the real behavior tree launcher (requires --force)')
    rn.add_argument('--subtrees', nargs='+', default=['cube'])
    rn.add_argument('--mode', choices=['sequence', 'parallel'], default='sequence')
    rn.add_argument('--force', action='store_true', help='Confirm you want to actually start the mission')

    return p.parse_args(argv)


def main(argv: List[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    args = parse_args(argv)

    if args.cmd == 'runbook':
        print_runbook()
        return 0

    if args.cmd == 'dry-run':
        return do_dry_run([s.lower() for s in args.subtrees], args.mode)

    if args.cmd == 'test':
        return run_tests(args.which)

    if args.cmd == 'run':
        return run_launcher([s.lower() for s in args.subtrees], args.mode, force=args.force)

    print('Unknown command')
    return 4


if __name__ == '__main__':
    rc = main()
    sys.exit(rc)
