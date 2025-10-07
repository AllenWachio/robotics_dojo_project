"""
Competition 2025 Behavior Library
==================================
All behavior nodes for autonomous competition mission.

Usage:
    from behaviors import MoveToPosition, ReadColorSensor, WaitForDiseaseDetection
"""

# Navigation behaviors
from .navigation import (
    MoveToPosition,
    StopRobot,
    ReverseDistance,
    Turn180Degrees,
)

# Color sensor behaviors
from .color_sensor import (
    ReadColorSensor,
    WaitForColorSensorClear,
)

# Camera behaviors
from .camera import (
    MonitorCameraForColor,
    VerifyColorMatch,
    WaitForCameraColorClear,
)

# Disease detection behaviors
from .disease_detection import (
    WaitForDiseaseDetection,
    LogDiseaseResult,
    CheckDiseaseDetectionRequired,
)

# Actuator behaviors
from .actuators import (
    ActivateCameraServo,
    ActivateTipperServo,
    ActivateConveyorBelt,
    ActivateStepperMotor,
    ResetTipperServo,
)

__all__ = [
    # Navigation
    'MoveToPosition',
    'StopRobot',
    'ReverseDistance',
    'Turn180Degrees',
    # Color Sensor
    'ReadColorSensor',
    'WaitForColorSensorClear',
    # Camera
    'MonitorCameraForColor',
    'VerifyColorMatch',
    'WaitForCameraColorClear',
    # Disease Detection
    'WaitForDiseaseDetection',
    'LogDiseaseResult',
    'CheckDiseaseDetectionRequired',
    # Actuators
    'ActivateCameraServo',
    'ActivateTipperServo',
    'ActivateConveyorBelt',
    'ActivateStepperMotor',
    'ResetTipperServo',
]
