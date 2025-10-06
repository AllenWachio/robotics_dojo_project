#!/usr/bin/env python3
"""
Potato Disease Detection Inference Engine
Uses PyTorch ResNet18 model for classification
Fixed paths using ament_index_python for proper package resource location
"""

import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
import os
from ament_index_python.packages import get_package_share_directory


class PotatoDiseaseModel:
    """
    Potato disease classification model using ResNet18.
    
    Classes:
        - Early_blight
        - Late_blight
        - Healthy
    """
    
    def __init__(self, model_name='model_ft_gpu.pth'):
        """
        Initialize the model with proper path resolution.
        
        Args:
            model_name (str): Name of the model file in the models directory
        """
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model_name = model_name
        self.model = None
        self.class_names = ['Early_blight', 'Late_blight', 'Healthy']
        
        # Define preprocessing transforms
        self.transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406],
                                 [0.229, 0.224, 0.225])
        ])
        
        # Load the model
        self._load_model()
    
    def _get_model_path(self):
        """
        Get the absolute path to the model file using ament_index.
        
        Returns:
            str: Absolute path to model file
            
        Raises:
            FileNotFoundError: If model file doesn't exist
        """
        try:
            # Get package installation directory
            package_share_dir = get_package_share_directory('rpi_camera_package')
            
            # Try share directory first (installed package)
            model_path = os.path.join(package_share_dir, 'models', self.model_name)
            
            if os.path.exists(model_path):
                return model_path
            
            # Fallback: Try source directory (development)
            # This handles the case when running from source without install
            import rpi_camera_package
            pkg_dir = os.path.dirname(rpi_camera_package.__file__)
            model_path = os.path.join(pkg_dir, 'models', self.model_name)
            
            if os.path.exists(model_path):
                return model_path
            
            raise FileNotFoundError(
                f"Model file '{self.model_name}' not found in package. "
                f"Searched locations:\n"
                f"  - {os.path.join(package_share_dir, 'models', self.model_name)}\n"
                f"  - {model_path}"
            )
            
        except Exception as e:
            raise FileNotFoundError(
                f"Could not locate model file '{self.model_name}': {str(e)}"
            )
    
    def _load_model(self):
        """
        Load the ResNet18 model with trained weights.
        
        Raises:
            FileNotFoundError: If model file not found
            RuntimeError: If model loading fails
        """
        try:
            # Get model path
            model_path = self._get_model_path()
            print(f"[PotatoDiseaseModel] Loading model from: {model_path}")
            
            # Create ResNet18 architecture
            self.model = models.resnet18(pretrained=False)
            
            # Modify final layer for 3 classes
            num_ftrs = self.model.fc.in_features
            self.model.fc = nn.Linear(num_ftrs, 3)
            
            # Load trained weights
            self.model.load_state_dict(
                torch.load(model_path, map_location=self.device)
            )
            
            # Move to device and set to evaluation mode
            self.model = self.model.to(self.device)
            self.model.eval()
            
            device_name = "GPU" if torch.cuda.is_available() else "CPU"
            print(f"[PotatoDiseaseModel] ✅ Model loaded successfully on {device_name}")
            print(f"[PotatoDiseaseModel] Classes: {', '.join(self.class_names)}")
            
        except FileNotFoundError as e:
            print(f"[PotatoDiseaseModel] ❌ Error: {e}")
            raise
        except Exception as e:
            print(f"[PotatoDiseaseModel] ❌ Failed to load model: {e}")
            raise RuntimeError(f"Model loading failed: {e}")
    
    def predict(self, image: Image.Image):
        """
        Run inference on a PIL image and return class name with confidence.
        
        Args:
            image (PIL.Image.Image): Input image
            
        Returns:
            tuple: (class_name: str, confidence: float)
                - class_name: Predicted disease class
                - confidence: Confidence score (0.0 to 1.0)
                
        Raises:
            ValueError: If image is invalid
            RuntimeError: If inference fails
        """
        if image is None:
            raise ValueError("Input image is None")
        
        if self.model is None:
            raise RuntimeError("Model not loaded. Cannot perform inference.")
        
        try:
            with torch.no_grad():
                # Preprocess image
                x = self.transform(image).unsqueeze(0).to(self.device)
                
                # Run inference
                outputs = self.model(x)
                
                # Get probabilities
                probabilities = torch.nn.functional.softmax(outputs, dim=1)
                confidence, pred_idx = torch.max(probabilities, 1)
                
                # Get class name and confidence
                class_name = self.class_names[pred_idx.item()]
                confidence_score = confidence.item()
                
                return class_name, confidence_score
                
        except Exception as e:
            raise RuntimeError(f"Inference failed: {e}")
    
    def get_class_names(self):
        """
        Get list of class names.
        
        Returns:
            list: List of class names
        """
        return self.class_names
    
    def is_loaded(self):
        """
        Check if model is loaded and ready.
        
        Returns:
            bool: True if model is loaded
        """
        return self.model is not None


# Module-level function for backward compatibility
def create_model(model_name='model_ft_gpu.pth'):
    """
    Factory function to create a PotatoDiseaseModel instance.
    
    Args:
        model_name (str): Name of the model file
        
    Returns:
        PotatoDiseaseModel: Initialized model instance
    """
    return PotatoDiseaseModel(model_name=model_name)
