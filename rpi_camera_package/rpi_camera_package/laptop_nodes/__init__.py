"""
Laptop-side processing nodes for rpi_camera_package.
Contains color detection, disease detection, and other image processing nodes.
"""

from .color_detection_node import ColorDetectionNode
from .disease_detection_node import DiseaseDetectionNode

__all__ = ['ColorDetectionNode', 'DiseaseDetectionNode']
