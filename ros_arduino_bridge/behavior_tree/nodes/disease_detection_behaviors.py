#!/usr/bin/env python3
"""
Disease Detection Behavior Nodes
=================================
Behaviors for potato disease detection using ML model
Topics: /inference_result (String from disease detection node)

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from std_msgs.msg import String
import time


class WaitForDiseaseDetection(py_trees.behaviour.Behaviour):
    """
    Wait for disease detection result from ML model.
    Subscribes to /inference_result topic published by disease_detection_node.
    
    Operation:
    1. Subscribe to /inference_result topic
    2. Wait for classification result
    3. Store result in blackboard['disease_detection_result']
    4. Return SUCCESS with result or timeout
    
    Expected results: 'Early Blight', 'Late Blight', 'Healthy'
    
    Returns: Always SUCCESS (timeout allowed for manual observation)
    Blackboard Output: 'disease_detection_result' = classification or 'timeout'
    """
    
    def __init__(self, name="WaitForDiseaseDetection", timeout=15.0):
        super().__init__(name)
        self.timeout = timeout
        self.node = None
        self.detection_sub = None
        self.detection_result = None
        self.start_time = None
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 subscription and blackboard"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to disease detection results
        self.detection_sub = self.node.create_subscription(
            String,
            '/inference_result',
            self.detection_callback,
            10
        )
        
        # Setup blackboard
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='disease_detection_result',
            access=py_trees.common.Access.WRITE
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def detection_callback(self, msg):
        """Store detection result"""
        self.detection_result = msg.data
    
    def initialise(self):
        """Start waiting for detection"""
        self.detection_result = None
        self.start_time = time.time()
        self.logger.info(f"{self.name}: 🔬 Waiting for disease detection result...")
    
    def update(self):
        """Check for detection result"""
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: ⏱️ Timeout - no detection result")
            self.logger.info(f"{self.name}: 🤚 Continuing with manual observation...")
            self.blackboard.disease_detection_result = 'timeout'
            return py_trees.common.Status.SUCCESS
        
        # Wait for result
        if self.detection_result is None:
            return py_trees.common.Status.RUNNING
        
        # Got result!
        self.logger.info(f"{self.name}: ✓ Detection result: {self.detection_result}")
        self.blackboard.disease_detection_result = self.detection_result
        
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class LogDiseaseResult(py_trees.behaviour.Behaviour):
    """
    Log the disease detection result for competition judges.
    Reads from blackboard and prints clearly formatted result.
    
    Returns: Always SUCCESS
    """
    
    def __init__(self, name="LogDiseaseResult"):
        super().__init__(name)
        self.blackboard = None
        self.logged = False
        
    def setup(self, **kwargs):
        """Initialize blackboard"""
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='disease_detection_result',
            access=py_trees.common.Access.READ
        )
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Reset logging flag"""
        self.logged = False
    
    def update(self):
        """Log the result"""
        if not self.logged:
            result = self.blackboard.get('disease_detection_result')
            
            print("\n" + "="*60)
            print("🔬 DISEASE DETECTION RESULT")
            print("="*60)
            print(f"   Classification: {result}")
            print("="*60 + "\n")
            
            self.logger.info(f"{self.name}: Disease result logged: {result}")
            self.logged = True
        
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Cleanup"""
        pass


class CheckDiseaseDetectionRequired(py_trees.behaviour.Behaviour):
    """
    Check if disease detection phase is required for this competition run.
    Can be configured via ROS parameter.
    
    Returns: SUCCESS (required) | FAILURE (skip disease detection)
    """
    
    def __init__(self, name="CheckDiseaseDetectionRequired"):
        super().__init__(name)
        self.node = None
        self.checked = False
        
    def setup(self, **kwargs):
        """Initialize node and parameter"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Declare parameter (default: True - disease detection required)
        self.node.declare_parameter('disease_detection_enabled', True)
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def initialise(self):
        """Reset check flag"""
        self.checked = False
    
    def update(self):
        """Check if disease detection is required"""
        if not self.checked:
            enabled = self.node.get_parameter('disease_detection_enabled').value
            
            if enabled:
                self.logger.info(f"{self.name}: ✓ Disease detection REQUIRED")
                self.checked = True
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.info(f"{self.name}: ⊗ Disease detection SKIPPED (disabled)")
                self.checked = True
                return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """Cleanup"""
        pass
