#!/usr/bin/env python3
"""
Motor Control Behavior Nodes
=============================
Behaviors for controlling servos, stepper motors, and conveyor belt
Topics: /camera_servo/command, /tipper_servo/command, /stepper/command

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from std_msgs.msg import Int32, String, Bool
import time


class ActivateCameraServo(py_trees.behaviour.Behaviour):
    """
    Adjust camera servo angle for disease detection.
    
    Args:
        target_angle: Servo angle in degrees (0-180)
            - 0° = looking down
            - 45° = angled view
            - 90° = looking forward
    
    Returns: SUCCESS (position reached or timeout)
    """
    
    def __init__(self, name="ActivateCameraServo", target_angle=45):
        super().__init__(name)
        self.target_angle = target_angle
        self.node = None
        self.servo_pub = None
        self.servo_feedback_sub = None
        self.current_angle = None
        self.command_sent = False
        self.start_time = None
        self.timeout = 3.0
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for servo command
        self.servo_pub = self.node.create_publisher(
            Int32,
            '/camera_servo/command',
            10
        )
        
        # Subscriber for feedback
        self.servo_feedback_sub = self.node.create_subscription(
            Int32,
            '/camera_servo/angle',
            self.feedback_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def feedback_callback(self, msg):
        """Receive current servo angle"""
        self.current_angle = msg.data
    
    def initialise(self):
        """Send servo command"""
        self.command_sent = False
        self.start_time = time.time()
        self.logger.info(f"{self.name}: 📷 Moving camera servo to {self.target_angle}°...")
    
    def update(self):
        """Monitor servo until position reached"""
        # Send command once
        if not self.command_sent:
            msg = Int32()
            msg.data = self.target_angle
            self.servo_pub.publish(msg)
            self.command_sent = True
        
        # Check if reached target
        if self.current_angle is not None:
            error = abs(self.current_angle - self.target_angle)
            if error <= 5:  # 5 degree tolerance
                self.logger.info(f"{self.name}: ✓ Camera at {self.current_angle}°")
                return py_trees.common.Status.SUCCESS
        
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: Timeout - assuming position reached")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class ActivateTipperServo(py_trees.behaviour.Behaviour):
    """
    Activate tipper servo to tilt robot back for offloading.
    
    Args:
        target_angle: Servo angle in degrees
            - 170° = normal position
            - 90° = tilted back (for offloading)
    
    Returns: SUCCESS (position reached or timeout)
    """
    
    def __init__(self, name="ActivateTipperServo", target_angle=90):
        super().__init__(name)
        self.target_angle = target_angle
        self.node = None
        self.servo_pub = None
        self.servo_feedback_sub = None
        self.current_angle = None
        self.command_sent = False
        self.start_time = None
        self.timeout = 3.0
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for servo command
        self.servo_pub = self.node.create_publisher(
            Int32,
            '/tipper_servo/command',
            10
        )
        
        # Subscriber for feedback
        self.servo_feedback_sub = self.node.create_subscription(
            Int32,
            '/tipper_servo/angle',
            self.feedback_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def feedback_callback(self, msg):
        """Receive current servo angle"""
        self.current_angle = msg.data
    
    def initialise(self):
        """Send servo command"""
        self.command_sent = False
        self.start_time = time.time()
        self.logger.info(f"{self.name}: 🎚️ Activating tipper servo to {self.target_angle}°...")
    
    def update(self):
        """Monitor servo until position reached"""
        # Send command once
        if not self.command_sent:
            msg = Int32()
            msg.data = self.target_angle
            self.servo_pub.publish(msg)
            self.command_sent = True
        
        # Check if reached target
        if self.current_angle is not None:
            error = abs(self.current_angle - self.target_angle)
            if error <= 5:  # 5 degree tolerance
                self.logger.info(f"{self.name}: ✓ Tipper at {self.current_angle}°")
                return py_trees.common.Status.SUCCESS
        
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: Timeout - assuming position reached")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class ActivateConveyorBelt(py_trees.behaviour.Behaviour):
    """
    Activate conveyor belt to offload cargo.
    
    Args:
        duration: Time to run conveyor (seconds)
        speed: PWM speed (0-255, or percentage 0-100)
    
    Command format sent to Arduino: "conveyor:speed:duration_ms"
    
    Returns: SUCCESS (after duration)
    """
    
    def __init__(self, name="ActivateConveyorBelt", duration=5.0, speed=200):
        super().__init__(name)
        self.duration = duration
        self.speed = speed
        self.node = None
        self.conveyor_pub = None
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for conveyor command
        self.conveyor_pub = self.node.create_publisher(
            String,
            '/stepper/command',  # Reusing stepper topic for conveyor
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Start conveyor"""
        self.start_time = time.time()
        
        # Send command
        cmd = String()
        cmd.data = f"conveyor:{self.speed}:{int(self.duration * 1000)}"
        self.conveyor_pub.publish(cmd)
        
        self.logger.info(f"{self.name}: 🚛 Conveyor running (speed={self.speed}, duration={self.duration}s)...")
    
    def update(self):
        """Wait for duration"""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            self.logger.info(f"{self.name}: ✓ Conveyor complete")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Stop conveyor"""
        if self.conveyor_pub:
            cmd = String()
            cmd.data = "conveyor:0:0"
            self.conveyor_pub.publish(cmd)


class ActivateStepperMotor(py_trees.behaviour.Behaviour):
    """
    Activate stepper motor for offloading (if using stepper instead of conveyor).
    
    Args:
        steps: Number of steps to move
        direction: 'forward' or 'reverse'
    
    Command format: "stepper:steps:direction"
    
    Returns: SUCCESS (after completion)
    """
    
    def __init__(self, name="ActivateStepperMotor", steps=200, direction='forward'):
        super().__init__(name)
        self.steps = steps
        self.direction = direction
        self.node = None
        self.stepper_pub = None
        self.stepper_status_sub = None
        self.is_active = True
        self.start_time = None
        self.timeout = 10.0
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Publisher for stepper command
        self.stepper_pub = self.node.create_publisher(
            String,
            '/stepper/command',
            10
        )
        
        # Subscriber for status
        self.stepper_status_sub = self.node.create_subscription(
            Bool,
            '/stepper/active',
            self.status_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def status_callback(self, msg):
        """Monitor stepper status"""
        self.is_active = msg.data
    
    def initialise(self):
        """Send stepper command"""
        self.start_time = time.time()
        self.is_active = True
        
        # Send command
        cmd = String()
        cmd.data = f"stepper:{self.steps}:{self.direction}"
        self.stepper_pub.publish(cmd)
        
        self.logger.info(f"{self.name}: ⚙️ Stepper running ({self.steps} steps, {self.direction})...")
    
    def update(self):
        """Wait for completion"""
        # Check if stepper finished
        if not self.is_active:
            self.logger.info(f"{self.name}: ✓ Stepper complete")
            return py_trees.common.Status.SUCCESS
        
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: Timeout - assuming complete")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Stop stepper"""
        if self.stepper_pub:
            cmd = String()
            cmd.data = "stepper:0:stop"
            self.stepper_pub.publish(cmd)


class ResetTipperServo(py_trees.behaviour.Behaviour):
    """
    Reset tipper servo to normal position after offloading.
    
    Returns: SUCCESS (position reached)
    """
    
    def __init__(self, name="ResetTipperServo", normal_angle=170):
        super().__init__(name)
        self.normal_angle = normal_angle
        self.node = None
        self.servo_pub = None
        self.command_sent = False
        self.start_time = None
        self.timeout = 3.0
        
    def setup(self, **kwargs):
        """Initialize publisher"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.servo_pub = self.node.create_publisher(
            Int32,
            '/tipper_servo/command',
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Reset servo"""
        self.command_sent = False
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Resetting tipper to {self.normal_angle}°...")
    
    def update(self):
        """Send reset command"""
        if not self.command_sent:
            msg = Int32()
            msg.data = self.normal_angle
            self.servo_pub.publish(msg)
            self.command_sent = True
        
        # Wait for timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.info(f"{self.name}: ✓ Tipper reset")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass
