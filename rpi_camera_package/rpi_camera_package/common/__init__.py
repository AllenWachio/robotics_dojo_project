"""
Common utilities shared between pi and laptop nodes.
"""

from .inference_engine import PotatoDiseaseModel, create_model

__all__ = ['PotatoDiseaseModel', 'create_model']
