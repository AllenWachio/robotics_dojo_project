#!/usr/bin/env python3
"""
Behavior Tree Nodes Package
============================
Clean, modular behavior nodes for autonomous robot competition

Usage:
    from behavior_tree.nodes import *
    # or
    from behavior_tree.nodes.color_sensor_behaviors import ReadColorSensor
    from behavior_tree.nodes.navigation_behaviors import MoveToPosition

Author: Robotics Dojo 2025
"""

# Color Sensor Behaviors
from .color_sensor_behaviors import (
    ReadColorSensor,
    WaitForColorSensorClear
)

# Camera Behaviors
from .camera_behaviors import (
    MonitorCameraForColor,
    WaitForCameraColorClear,
    VerifyColorMatch
)

# Disease Detection Behaviors
from .disease_detection_behaviors import (
    WaitForDiseaseDetection,
    LogDiseaseResult,
    CheckDiseaseDetectionRequired
)

# Navigation Behaviors
from .navigation_behaviors import (
    MoveToPosition,
    ReverseDistance,
    Turn180Degrees,
    StopRobot,
    MoveRelativeDistance
)

# Motor Control Behaviors
from .motor_control_behaviors import (
    ActivateCameraServo,
    ActivateTipperServo,
    ActivateConveyorBelt,
    ActivateStepperMotor,
    ResetTipperServo
)

__all__ = [
    # Color Sensor
    'ReadColorSensor',
    'WaitForColorSensorClear',
    
    # Camera
    'MonitorCameraForColor',
    'WaitForCameraColorClear',
    'VerifyColorMatch',
    
    # Disease Detection
    'WaitForDiseaseDetection',
    'LogDiseaseResult',
    'CheckDiseaseDetectionRequired',
    
    # Navigation
    'MoveToPosition',
    'ReverseDistance',
    'Turn180Degrees',
    'StopRobot',
    'MoveRelativeDistance',
    
    # Motor Control
    'ActivateCameraServo',
    'ActivateTipperServo',
    'ActivateConveyorBelt',
    'ActivateStepperMotor',
    'ResetTipperServo',
]
