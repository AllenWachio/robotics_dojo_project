# Potato Disease Detection - User Guide

## 🌱 Overview

The potato disease detection system uses a PyTorch ResNet18 deep learning model to classify potato leaf health status from camera images in real-time.

### **Detection Classes**

- 🟢 **Healthy** - Normal healthy potato leaves
- 🟠 **Early_blight** - Early stage blight disease
- 🔴 **Late_blight** - Late stage blight disease

---

## 🏗️ Architecture

```
┌─────────────────┐         Network          ┌──────────────────────────┐
│  RASPBERRY PI   │◄──────────────────────────►│        LAPTOP            │
│                 │                            │                          │
│  v4l2_camera    │  /camera/image_raw/        │  ┌────────────────────┐  │
│      ↓          │      compressed            │  │ Disease Detection  │  │
│  Publishes      │──────────────────────────→ │  │ Node               │  │
│  Images         │                            │  │ - PyTorch ResNet18 │  │
│                 │                            │  │ - Rate limited     │  │
│                 │                            │  │ - QoS optimized    │  │
│                 │                            │  └────────────────────┘  │
│                 │                            │           ↓              │
│                 │                            │  /disease_detection/     │
│                 │                            │     - result             │
│                 │                            │     - annotated_image    │
└─────────────────┘                            └──────────────────────────┘
```

**Key Design Features:**

- ⚡ Heavy ML inference runs on **Laptop** (not Pi)
- 📦 Uses compressed images over network
- 🎯 Rate-limited inference (configurable Hz)
- 🔄 Proper QoS profiles for real-time streaming
- 📊 Confidence scores with each prediction

---

## 📦 Dependencies

### **Required (Laptop Side)**

Install PyTorch and related packages:

```bash
# CPU version (most common)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# GPU version (if you have CUDA)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# PIL for image processing
pip3 install pillow

# Ensure NumPy < 2.0 for cv_bridge compatibility
pip3 install "numpy<2.0" --force-reinstall
```

### **Model File**

The trained PyTorch model (`model_ft_gpu.pth`, ~43MB) is automatically installed with the package.

---

## 🚀 Usage

### **Basic Usage**

#### **1. On Raspberry Pi** (Camera)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rpi_camera_package camera_compressed.launch.py pi
```

#### **2. On Laptop** (Disease Detection Only)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rpi_camera_package disease_detection.launch.py laptop
```

#### **3. On Laptop** (BOTH Color + Disease Detection)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rpi_camera_package full_processing.launch.py laptop
```

---

### **Advanced Usage with Parameters**

#### **Adjust Inference Rate** (CPU Optimization)

```bash
# Process 1 frame per second (default)
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=1.0

# Process 0.5 frames per second (every 2 seconds - saves CPU)
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=0.5

# Process 2 frames per second (more frequent detection)
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=2.0
```

#### **Set Confidence Threshold**

```bash
# Report all detections (default)
ros2 launch rpi_camera_package disease_detection.launch.py laptop confidence_threshold:=0.0

# Only report if 50%+ confident
ros2 launch rpi_camera_package disease_detection.launch.py laptop confidence_threshold:=0.5

# Only report if 80%+ confident
ros2 launch rpi_camera_package disease_detection.launch.py laptop confidence_threshold:=0.8
```

#### **Disable Display Window** (Headless Mode)

```bash
ros2 launch rpi_camera_package disease_detection.launch.py laptop display:=false
```

#### **Use Raw Images** (Not Compressed)

```bash
ros2 launch rpi_camera_package disease_detection.launch.py laptop use_compressed:=false
```

---

## 📊 Topics

### **Subscribed Topics**

| Topic                          | Type                        | Description                                |
| ------------------------------ | --------------------------- | ------------------------------------------ |
| `/camera/image_raw/compressed` | sensor_msgs/CompressedImage | Compressed camera feed (default)           |
| `/camera/image_raw`            | sensor_msgs/Image           | Raw camera feed (if use_compressed:=false) |

### **Published Topics**

| Topic                                | Type              | Description                           |
| ------------------------------------ | ----------------- | ------------------------------------- |
| `/disease_detection/result`          | std_msgs/String   | Classification result with confidence |
| `/disease_detection/annotated_image` | sensor_msgs/Image | Image with overlay showing detection  |

### **Example Messages**

**Result Topic:**

```
data: "Disease: Healthy | Confidence: 98.5%"
data: "Disease: Early_blight | Confidence: 87.3%"
data: "Disease: Late_blight | Confidence: 92.1%"
```

---

## 🔧 Configuration

### **Parameters File**

`config/laptop/disease_detection_params.yaml`

```yaml
/disease_detection_node:
  ros__parameters:
    use_compressed: true # Use compressed images
    display_output: true # Show OpenCV window
    publish_annotated: true # Publish annotated images
    inference_rate: 1.0 # Inference frequency (Hz)
    confidence_threshold: 0.0 # Min confidence (0.0-1.0)
    model_name: "model_ft_gpu.pth" # Model filename
```

---

## 🖼️ Visual Output

The annotated image display shows:

- **Status Banner** (top): Disease name with color coding
  - 🟢 Green = Healthy
  - 🟠 Orange = Early Blight
  - 🔴 Red = Late Blight
- **Confidence Score**: Percentage confidence
- **Frame Counter**: Number of frames processed
- **Colored Border**: Matches disease status

---

## 📈 Performance

### **Typical Performance (Laptop)**

- **Inference Time**: 100-300ms per image (CPU)
- **Inference Time**: 20-50ms per image (GPU)
- **Recommended Rate**: 1 Hz (once per second)
- **CPU Usage**: ~30-50% (at 1 Hz)
- **Memory**: ~500MB (model loaded)

### **Network Performance**

- **Bandwidth**: ~1-3 Mbps (compressed JPEG)
- **Latency**: 50-150ms (Pi → Laptop)
- **Total Delay**: ~200-500ms (capture to result)

---

## 🧪 Testing

### **1. Monitor Detection Results**

```bash
ros2 topic echo /disease_detection/result
```

### **2. View Annotated Images**

```bash
ros2 run rqt_image_view rqt_image_view /disease_detection/annotated_image
```

### **3. Check Statistics**

Look for periodic stats in the terminal:

```
[disease_detection_node] 📊 Stats: 30 frames, 5 inferences in 5s (~6.0 FPS, ~1.0 Inf/s)
```

### **4. Test Different Leaves**

Show the camera:

- Healthy potato leaves → Should detect "Healthy"
- Diseased leaves → Should detect "Early_blight" or "Late_blight"

---

## 🐛 Troubleshooting

### **Issue: "Failed to load model"**

**Problem**: PyTorch not installed or model file missing

**Solutions**:

```bash
# Install PyTorch
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Verify model exists
ls ~/ros2_ws/install/rpi_camera_package/share/rpi_camera_package/models/

# Rebuild package
cd ~/ros2_ws
colcon build --packages-select rpi_camera_package
```

### **Issue: "Import torch could not be resolved"**

**Problem**: PyTorch not in Python path

**Solution**:

```bash
# Make sure you install in the correct Python environment
python3 -c "import torch; print(torch.__version__)"

# If error, reinstall
pip3 install torch torchvision
```

### **Issue: "No frames received"**

**Problem**: Camera not publishing or network issue

**Solutions**:

```bash
# Check if camera is publishing
ros2 topic hz /camera/image_raw/compressed

# Check ROS2 discovery
ros2 node list

# Restart ROS2 daemon
ros2 daemon stop && ros2 daemon start
```

### **Issue: High CPU Usage**

**Problem**: Inference rate too high

**Solution**:

```bash
# Reduce inference rate
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=0.5
```

### **Issue: Low Confidence Scores**

**Problem**: Poor lighting, wrong angle, or non-potato leaf

**Solutions**:

- Ensure good lighting on the leaf
- Hold leaf flat facing camera
- Get closer to the leaf (fill frame)
- Use actual potato leaves (not other plants)

---

## 🎯 Best Practices

1. **Inference Rate**: Start with 1.0 Hz, adjust based on CPU
2. **Lighting**: Ensure consistent, bright lighting
3. **Distance**: Fill 50-80% of frame with leaf
4. **Angle**: Hold leaf flat, perpendicular to camera
5. **Background**: Plain background works best
6. **Network**: Use 5GHz WiFi or Ethernet for best performance

---

## 🔬 Model Information

### **Architecture**

- **Base Model**: ResNet18 (CNN)
- **Input Size**: 224x224 pixels
- **Preprocessing**: Resize → CenterCrop → Normalize
- **Classes**: 3 (Healthy, Early_blight, Late_blight)
- **Training**: Transfer learning (fine-tuned)

### **Model File**

- **Filename**: `model_ft_gpu.pth`
- **Size**: ~43 MB
- **Location**: `share/rpi_camera_package/models/`
- **Format**: PyTorch state dict

---

## 📚 Related Documentation

- [README.md](README.md) - Main package documentation
- [QUICKSTART.md](QUICKSTART.md) - Quick reference guide
- [TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) - Verification steps

---

## 💡 Tips

- **Batch Testing**: Use `full_processing.launch.py` to run both color and disease detection
- **Save Results**: Use `ros2 bag record` to save detections for analysis
- **Remote Monitoring**: View results over network without display window
- **Multiple Cameras**: Can run multiple instances with different cameras

---

**Last Updated**: October 5, 2025
