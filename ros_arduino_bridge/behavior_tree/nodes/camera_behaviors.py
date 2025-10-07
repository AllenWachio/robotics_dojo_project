#!/usr/bin/env python3
"""
Camera Behavior Nodes
=====================
Behaviors for Pi Camera color detection and monitoring
Topics: /camera/color_detection (String)

Note: The laptop_nodes/color_detection_node.py should publish detected colors

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from std_msgs.msg import String
import time


class MonitorCameraForColor(py_trees.behaviour.Behaviour):
    """
    Monitor Pi camera for color detection matching the cargo color.
    Used during navigation to delivery zone - stops when matching color found.
    
    Operation:
    1. Read detected_color from blackboard (from cargo sensor)
    2. Subscribe to /camera/color_detection topic
    3. Compare incoming detections with target color
    4. Return SUCCESS when match found
    
    Returns: SUCCESS (color matched) | RUNNING (still looking)
    Blackboard Input: 'detected_color' (target to match)
    """
    
    def __init__(self, name="MonitorCameraForColor", timeout=60.0):
        super().__init__(name)
        self.timeout = timeout
        self.node = None
        self.camera_sub = None
        self.target_color = None
        self.latest_detection = None
        self.start_time = None
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 subscription and blackboard"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to camera color detection
        self.camera_sub = self.node.create_subscription(
            String,
            '/camera/color_detection',
            self.camera_callback,
            10
        )
        
        # Setup blackboard to read target color
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='detected_color',
            access=py_trees.common.Access.READ
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def camera_callback(self, msg):
        """Store latest camera detection"""
        self.latest_detection = msg.data.lower()
    
    def initialise(self):
        """Start monitoring"""
        self.latest_detection = None
        self.start_time = time.time()
        
        # Get target color from blackboard
        self.target_color = self.blackboard.get('detected_color')
        
        if self.target_color == 'unknown':
            self.logger.warning(f"{self.name}: ⚠️ Target color is 'unknown' - will monitor but may need manual stop")
        
        self.logger.info(f"{self.name}: 📷 Monitoring camera for '{self.target_color}' color...")
    
    def update(self):
        """Check if camera detected target color"""
        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            self.logger.warning(f"{self.name}: ⏱️ Timeout - color not detected")
            # Don't fail - allow manual intervention
            return py_trees.common.Status.SUCCESS
        
        # Wait for detection
        if self.latest_detection is None:
            return py_trees.common.Status.RUNNING
        
        # Check if matches target
        if self.target_color == 'unknown':
            # Special case - can't auto-detect, need manual intervention
            return py_trees.common.Status.RUNNING
        
        if self.latest_detection == self.target_color:
            self.logger.info(f"{self.name}: ✓ Camera detected '{self.target_color}'! Stopping here.")
            return py_trees.common.Status.SUCCESS
        
        # Keep monitoring
        if elapsed % 5.0 < 0.1:  # Log every 5 seconds
            self.logger.info(f"{self.name}: Looking for '{self.target_color}'... (last seen: {self.latest_detection})")
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class WaitForCameraColorClear(py_trees.behaviour.Behaviour):
    """
    Wait until camera no longer detects the target color.
    Used to verify robot has moved away from delivery zone.
    
    Returns: SUCCESS (color cleared) | RUNNING (still detecting)
    """
    
    def __init__(self, name="WaitForCameraColorClear", timeout=10.0):
        super().__init__(name)
        self.timeout = timeout
        self.node = None
        self.camera_sub = None
        self.target_color = None
        self.latest_detection = None
        self.start_time = None
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 subscription"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.camera_sub = self.node.create_subscription(
            String,
            '/camera/color_detection',
            self.camera_callback,
            10
        )
        
        # Get target color from blackboard
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='detected_color',
            access=py_trees.common.Access.READ
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def camera_callback(self, msg):
        """Store latest detection"""
        self.latest_detection = msg.data.lower()
    
    def initialise(self):
        """Start monitoring"""
        self.latest_detection = None
        self.start_time = time.time()
        self.target_color = self.blackboard.get('detected_color')
        self.logger.info(f"{self.name}: Waiting for '{self.target_color}' to clear from view...")
    
    def update(self):
        """Check if color no longer detected"""
        # Timeout = success (moved away)
        if time.time() - self.start_time > self.timeout:
            self.logger.info(f"{self.name}: ✓ Timeout reached, assuming clear")
            return py_trees.common.Status.SUCCESS
        
        # No data yet
        if self.latest_detection is None:
            return py_trees.common.Status.RUNNING
        
        # Check if still detecting target
        if self.latest_detection == self.target_color:
            return py_trees.common.Status.RUNNING
        
        # Cleared!
        self.logger.info(f"{self.name}: ✓ Camera no longer detecting '{self.target_color}'")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class VerifyColorMatch(py_trees.behaviour.Behaviour):
    """
    Verify that current camera detection matches the cargo color.
    Used before offloading to ensure robot is at correct delivery zone.
    
    Returns: SUCCESS (colors match) | FAILURE (wrong zone!)
    """
    
    def __init__(self, name="VerifyColorMatch", timeout=5.0):
        super().__init__(name)
        self.timeout = timeout
        self.node = None
        self.camera_sub = None
        self.target_color = None
        self.latest_detection = None
        self.start_time = None
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 subscription"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.camera_sub = self.node.create_subscription(
            String,
            '/camera/color_detection',
            self.camera_callback,
            10
        )
        
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='detected_color',
            access=py_trees.common.Access.READ
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def camera_callback(self, msg):
        """Store latest detection"""
        self.latest_detection = msg.data.lower()
    
    def initialise(self):
        """Start verification"""
        self.latest_detection = None
        self.start_time = time.time()
        self.target_color = self.blackboard.get('detected_color')
        self.logger.info(f"{self.name}: 🔍 Verifying delivery zone matches '{self.target_color}'...")
    
    def update(self):
        """Check if colors match"""
        # Timeout
        if time.time() - self.start_time > self.timeout:
            if self.target_color == 'unknown':
                # Manual loading - assume correct
                self.logger.info(f"{self.name}: ✓ Manual loading - assuming correct zone")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"{self.name}: ❌ Timeout - cannot verify color match")
                return py_trees.common.Status.FAILURE
        
        # Wait for detection
        if self.latest_detection is None:
            return py_trees.common.Status.RUNNING
        
        # Verify match
        if self.latest_detection == self.target_color:
            self.logger.info(f"{self.name}: ✓ Color match verified: '{self.target_color}'")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.error(f"{self.name}: ❌ Color mismatch! Expected '{self.target_color}', got '{self.latest_detection}'")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """Cleanup"""
        pass
