#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import sys

def print_same_line(msg1, msg2):
    sys.stdout.write("\033[F\033[F")  # カーソルを2行上に移動
    sys.stdout.write(f"{msg1}\n")
    sys.stdout.write(f"{msg2}\n")
    sys.stdout.flush()

# ---- Task 1 & Task 2 ----
class PrintHello(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHello"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hello")
            self.executed = True
        return py_trees.common.Status.SUCCESS


class PrintHi(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHi"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hi")
            self.executed = True
        return py_trees.common.Status.SUCCESS


class MoveToPosition(py_trees.behaviour.Behaviour):
    # Shared AMCL pose data across all instances
    global_x = 0.0
    global_y = 0.0
    global_yaw = 0.0
    pose_initialized = False
    pose_sub = None  # Single subscription shared across instances

    def __init__(self, name, target_x, target_y, tolerance=1.0):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.tolerance = tolerance
        self.action_client = None
        self.goal_handle = None
        self.result_future = None
        self.goal_sent = False
        self.completed = False
        self.goal_status = None
        self.param_client = None
        self.tolerance_set = False

    def setup(self, **kwargs):
        node = kwargs.get('node')
        if node:
            self.node = node
            # Create NavigateToPose action client
            self.action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

            # Create parameter client for controller_server
            self.param_client = node.create_client(
                SetParameters, '/controller_server/set_parameters')

            # Create single shared AMCL pose subscription
            if MoveToPosition.pose_sub is None:
                MoveToPosition.pose_sub = node.create_subscription(
                    PoseWithCovarianceStamped, '/amcl_pose',
                    MoveToPosition.shared_pose_callback, 10)
                print(f"AMCL pose subscription created")
            print(f"{self.name}: Setup completed")

    @classmethod
    def shared_pose_callback(cls, msg):
        import math
        # AMCL provides position in map frame directly
        cls.global_x = msg.pose.pose.position.x
        cls.global_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        cls.global_yaw = math.atan2(siny_cosp, cosy_cosp)

        cls.pose_initialized = True

    def update(self):
        import math

        # If already completed, don't run again
        if self.completed:
            return py_trees.common.Status.SUCCESS

        # Wait for AMCL pose data
        if not MoveToPosition.pose_initialized:
            print(f"{self.name}: Waiting for AMCL pose data...")
            return py_trees.common.Status.RUNNING

        # Set tolerance parameter before sending goal
        if not self.tolerance_set:
            # Wait for parameter service
            if not self.param_client.wait_for_service(timeout_sec=1.0):
                print(f"{self.name}: Waiting for parameter service...")
                return py_trees.common.Status.RUNNING

            # Set xy_goal_tolerance for both goal_checker and controller
            request = SetParameters.Request()

            # Create parameter for general_goal_checker
            param1 = Parameter()
            param1.name = 'general_goal_checker.xy_goal_tolerance'
            param1.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.tolerance)

            # Create parameter for FollowPath controller
            param2 = Parameter()
            param2.name = 'FollowPath.xy_goal_tolerance'
            param2.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.tolerance)

            request.parameters = [param1, param2]

            # Send parameter update request
            future = self.param_client.call_async(request)

            # Wait for response (with callback)
            def param_callback(future):
                try:
                    response = future.result()
                    if response and len(response.results) > 0:
                        if all(result.successful for result in response.results):
                            print(f"{self.name}: Set goal tolerance to {self.tolerance}m")
                            self.tolerance_set = True
                        else:
                            print(f"{self.name}: Warning - Some parameters failed to set")
                            self.tolerance_set = True  # Continue anyway
                except Exception as e:
                    print(f"{self.name}: Parameter setting error: {e}")
                    self.tolerance_set = True  # Continue anyway

            future.add_done_callback(param_callback)
            return py_trees.common.Status.RUNNING

        # Send goal if not sent yet
        if not self.goal_sent:
            # Wait for action server
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                print(f"{self.name}: Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING

            # Create goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = make_pose(self.target_x, self.target_y, yaw=0.0, frame="map")

            # Send goal
            print(f"{self.name}: Sending goal to ({self.target_x}, {self.target_y})")
            send_goal_future = self.action_client.send_goal_async(goal_msg)

            # Use callback to handle goal acceptance and request result
            def goal_response_callback(future):
                self.goal_handle = future.result()
                if not self.goal_handle.accepted:
                    print(f"{self.name}: Goal rejected!")
                    self.goal_status = 'rejected'
                else:
                    print(f"{self.name}: Goal accepted, navigating...")
                    # Request the result
                    self.result_future = self.goal_handle.get_result_async()
                    self.result_future.add_done_callback(self.result_callback)

            send_goal_future.add_done_callback(goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        # Wait for goal handle
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING

        # Check if goal was rejected
        if self.goal_status == 'rejected':
            print(f"{self.name}: Navigation failed (goal rejected)")
            return py_trees.common.Status.FAILURE

        # Check current distance to target for monitoring
        current_x = MoveToPosition.global_x
        current_y = MoveToPosition.global_y
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)

        print(f"{self.name}: Current({current_x:.2f}, {current_y:.2f}) -> Target({self.target_x}, {self.target_y}), Distance: {distance:.2f}m")

        # Wait for Nav2 to complete navigation
        if self.goal_status == 'succeeded':
            print(f"{self.name}: Navigation completed! Final distance: {distance:.2f}m")
            self.completed = True
            return py_trees.common.Status.SUCCESS
        elif self.goal_status == 'aborted' or self.goal_status == 'canceled':
            print(f"{self.name}: Navigation {self.goal_status}!")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def result_callback(self, future):
        """Callback for when the action completes"""
        result = future.result()
        status = result.status

        # status codes: UNKNOWN=0, ACCEPTED=1, EXECUTING=2, CANCELING=3, SUCCEEDED=4, CANCELED=5, ABORTED=6
        if status == 4:  # SUCCEEDED
            self.goal_status = 'succeeded'
        elif status == 6:  # ABORTED
            self.goal_status = 'aborted'
        elif status == 5:  # CANCELED
            self.goal_status = 'canceled'
        else:
            self.goal_status = f'unknown({status})'


# ---- Helper for goals ----
def make_pose(x, y, yaw=0.0, frame="map"):
    import math
    pose = PoseStamped()
    pose.header.frame_id = frame
    # Leave timestamp as zero to use current time
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Convert yaw to quaternion
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    print(f"Goal created: x={x}, y={y}, yaw={yaw}, frame={frame}")
    return pose


# ---- Tree Definition ----
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # Move to specific coordinates
    # ⚠️ IMPORTANT: Change these coordinates to match YOUR map!
    # Use RViz "2D Goal Pose" to find valid coordinates on your gamefield map
    # Current coordinates are EXAMPLES - they may not work on your map!
    
    # Example: Small test mission - adjust to match your map
    move1 = MoveToPosition("MoveToPoint1", 0.5, 0.5, tolerance=0.2)  # Short distance for testing
    task1 = PrintHello("Task1")
    move2 = MoveToPosition("MoveToPoint2", 0.5, -0.5, tolerance=0.2)  # Return close to start
    task2 = PrintHi("Task2")

    # Add to sequence
    root.add_children([move1, task1, move2, task2])
    return root


# ---- Main ----
def main():
    rclpy.init()

    root = create_root()

    # ROS2 node wrapper
    node = py_trees_ros.trees.BehaviourTree(root)

    try:
        node.setup(timeout=10.0, node=node.node)  # Pass the ROS node to behaviors

        # Publish initial pose to AMCL (set robot's starting position on the map)
        initial_pose_pub = node.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Wait for publisher to be ready
        import time
        time.sleep(1.0)

        # Create initial pose message (robot starts at origin of map)
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = node.node.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0  # No rotation

        # Publish multiple times to ensure it's received
        for _ in range(5):
            initial_pose_pub.publish(initial_pose)
            time.sleep(0.1)

        print("Initial pose published to AMCL")

        # Execute the tree with regular timer
        print("Behavior tree starting...")

        tree_completed = False  # Track completion state

        def tick_tree():
            nonlocal tree_completed

            if tree_completed:
                return  # Don't execute if already completed

            node.tick_tock(period_ms=500)  # Match timer interval

            if node.root.status == py_trees.common.Status.SUCCESS:
                print("Behavior tree completed successfully!")
                tree_completed = True  # Mark as completed
                return
            elif node.root.status == py_trees.common.Status.FAILURE:
                print("Behavior tree failed!")
                tree_completed = True  # Mark as completed
                return

        # Create a regular timer to execute the tree
        _timer = node.node.create_timer(0.5, tick_tree)  # Execute every 0.5 seconds

        print("Behavior tree is running...")
        rclpy.spin(node.node)
    except RuntimeError as e:
        print(f"Setup failed: {e}")
        print("Make sure Nav2 is running: ros2 launch nav2_bringup navigation_launch.py")
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
