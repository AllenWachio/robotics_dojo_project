import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image


class PotatoDiseaseModel:
    def __init__(self):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        
        # Load ResNet18 and modify final layer
        self.model = models.resnet18(pretrained=False) ## Should also test with pretrained=True
        num_ftrs = self.model.fc.in_features
        self.model.fc = nn.Linear(num_ftrs, 3)  # 3 classes
        model_path = './src/rdj2025_potato_disease_detection/models/model_ft_gpu.pth'
        self.model.load_state_dict(torch.load(model_path, map_location="cpu"))
        self.model = self.model.to(self.device)
        self.model.eval()

        self.class_names = ['Early_blight', 'Late_blight', 'Healthy']

        # Define preprocessing
        self.transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406],
                                 [0.229, 0.224, 0.225])
        ])

    def predict(self, image: Image.Image) -> str:
        """Run inference on a PIL image and return class name."""
        with torch.no_grad():
            x = self.transform(image).unsqueeze(0).to(self.device)
            outputs = self.model(x)
            _, pred_idx = torch.max(outputs, 1)
            return self.class_names[pred_idx.item()]
