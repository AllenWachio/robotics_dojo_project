#!/usr/bin/env python3
"""
Test Mission: Room Mapping with Incremental Navigation

This demonstrates:
1. Mapping a room (using SLAM)
2. Sending robot to positions using IncrementalMove
3. Letting py_trees handle movement, obstacle avoidance, and task execution
4. Coexistence with traditional MoveToPosition

Author: Robotics Dojo 2025
Date: October 6, 2025
"""

import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped
import sys
import time

# Import both movement behaviors
from robot_navigation_bt import MoveToPosition, PrintHello, PrintHi
from incremental_move_behavior import IncrementalMove


class WaitForMapping(py_trees.behaviour.Behaviour):
    """
    Behavior that waits for a specified time to allow mapping.
    Simulates room exploration/mapping phase.
    """
    def __init__(self, name="WaitForMapping", duration_seconds=5.0):
        super().__init__(name)
        self.duration = duration_seconds
        self.start_time = None
        
    def initialise(self):
        """Start the timer."""
        self.logger.info(f"\n{'='*60}")
        self.logger.info(f"{self.name}: Mapping room for {self.duration}s...")
        self.logger.info(f"{'='*60}")
        self.start_time = time.time()
        
    def update(self):
        """Check if duration has elapsed."""
        if self.start_time is None:
            return py_trees.common.Status.RUNNING
            
        elapsed = time.time() - self.start_time
        remaining = self.duration - elapsed
        
        if remaining > 0:
            # Log progress every second
            if int(elapsed) != int(elapsed - 0.1):  # Only log at second boundaries
                self.logger.info(f"{self.name}: Mapping... {remaining:.0f}s remaining")
            return py_trees.common.Status.RUNNING
        else:
            self.logger.info(f"{self.name}: ✅ Mapping phase complete!")
            return py_trees.common.Status.SUCCESS


class PerformTask(py_trees.behaviour.Behaviour):
    """
    Simulates performing a task at a location.
    Could be: taking photo, picking object, scanning area, etc.
    """
    def __init__(self, name, task_description="Task", duration=2.0):
        super().__init__(name)
        self.task_description = task_description
        self.duration = duration
        self.executed = False
        self.start_time = None
        
    def initialise(self):
        """Start the task."""
        self.logger.info(f"\n{'*'*60}")
        self.logger.info(f"{self.name}: Starting task - {self.task_description}")
        self.logger.info(f"{'*'*60}")
        self.start_time = time.time()
        self.executed = False
        
    def update(self):
        """Execute the task."""
        if not self.executed:
            elapsed = time.time() - self.start_time
            if elapsed < self.duration:
                remaining = self.duration - elapsed
                self.logger.info(f"{self.name}: Executing... {remaining:.1f}s")
                return py_trees.common.Status.RUNNING
            else:
                self.logger.info(f"{self.name}: ✅ Task completed: {self.task_description}")
                self.executed = True
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.SUCCESS


def create_test_mission():
    """
    Create a test behavior tree demonstrating incremental navigation.
    
    Mission Flow:
    1. Wait for mapping (simulated)
    2. Move to Position A incrementally
    3. Perform task at Position A
    4. Move to Position B incrementally
    5. Perform task at Position B
    6. Return to start using traditional MoveToPosition (comparison)
    
    This demonstrates:
    - IncrementalMove for precise, monitored movement
    - Task execution at waypoints
    - Coexistence with MoveToPosition
    - Obstacle avoidance and recovery
    """
    
    root = py_trees.composites.Sequence("TestMission", memory=True)
    
    # Phase 1: Mapping (simulated wait)
    mapping = WaitForMapping("MapRoom", duration_seconds=3.0)
    
    # Phase 2: Navigate to Position A (using IncrementalMove)
    # Test with small, safe coordinates first
    move_to_a = IncrementalMove(
        name="MoveToPointA",
        goal_x=0.5,
        goal_y=0.5,
        step_size=0.2,      # 20cm steps
        tolerance=0.15,     # 15cm acceptance
        max_retries=3
    )
    
    # Phase 3: Perform task at Position A
    task_a = PerformTask(
        "TaskAtA",
        task_description="Scan area for objects",
        duration=2.0
    )
    
    # Phase 4: Navigate to Position B (using IncrementalMove)
    move_to_b = IncrementalMove(
        name="MoveToPointB",
        goal_x=0.5,
        goal_y=-0.5,
        step_size=0.2,
        tolerance=0.15,
        max_retries=3
    )
    
    # Phase 5: Perform task at Position B
    task_b = PerformTask(
        "TaskAtB",
        task_description="Take photograph",
        duration=2.0
    )
    
    # Phase 6: Return to start using traditional MoveToPosition (for comparison)
    # This demonstrates coexistence of both behaviors
    return_home = MoveToPosition(
        "ReturnHome",
        target_x=0.0,
        target_y=0.0,
        tolerance=0.2
    )
    
    # Phase 7: Final task
    final_task = PerformTask(
        "FinalTask",
        task_description="Mission complete - save data",
        duration=1.0
    )
    
    # Add all behaviors to sequence
    root.add_children([
        mapping,
        move_to_a,
        task_a,
        move_to_b,
        task_b,
        return_home,
        final_task
    ])
    
    return root


def create_simple_test():
    """
    Simplified test for initial testing.
    Just moves to one position and back.
    """
    root = py_trees.composites.Sequence("SimpleTest", memory=True)
    
    # Print start
    start = PrintHello("StartMission")
    
    # Move to test position
    move_out = IncrementalMove(
        name="MoveOut",
        goal_x=0.5,
        goal_y=0.5,
        step_size=0.2,
        tolerance=0.15
    )
    
    # Simple task
    task = PerformTask("TestTask", "Quick test", duration=1.0)
    
    # Move back
    move_back = IncrementalMove(
        name="MoveBack",
        goal_x=0.0,
        goal_y=0.0,
        step_size=0.2,
        tolerance=0.15
    )
    
    # Print end
    end = PrintHi("EndMission")
    
    root.add_children([start, move_out, task, move_back, end])
    return root


def main(args=None):
    """Main function to run the test mission."""
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node
    node = rclpy.create_node('test_incremental_navigation')
    
    print("\n" + "="*60)
    print("TEST MISSION: Incremental Navigation")
    print("="*60)
    print("\nChoose mission:")
    print("  1. Simple Test (recommended for first test)")
    print("  2. Full Test Mission (mapping + tasks)")
    print("="*60)
    
    # For now, default to simple test
    # You can modify this to add user input if needed
    use_simple = True  # Set to False for full mission
    
    if use_simple:
        print("Running: Simple Test")
        root = create_simple_test()
    else:
        print("Running: Full Test Mission")
        root = create_test_mission()
    
    print("="*60 + "\n")
    
    # Setup behavior tree
    try:
        # Setup all behaviors with the node
        for behavior in py_trees.trees.BehaviourTree(root).root.iterate():
            if hasattr(behavior, 'setup'):
                behavior.setup(node=node)
        
        # Create behavior tree
        tree = py_trees.trees.BehaviourTree(root)
        
        print("✓ Behavior tree initialized")
        print("✓ Starting mission execution...\n")
        
        # Execution loop
        rate = node.create_rate(10)  # 10 Hz
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            tree.tick()
            
            # Check if tree completed
            if root.status == py_trees.common.Status.SUCCESS:
                print("\n" + "="*60)
                print("✅ MISSION COMPLETED SUCCESSFULLY!")
                print("="*60)
                break
            elif root.status == py_trees.common.Status.FAILURE:
                print("\n" + "="*60)
                print("❌ MISSION FAILED")
                print("="*60)
                break
            
            try:
                rate.sleep()
            except:
                pass
        
    except KeyboardInterrupt:
        print("\n⚠️ Mission interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
