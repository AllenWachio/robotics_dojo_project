# RPI Camera Package - Distributed System

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Distributed camera processing system for ROS2 Humble with Raspberry Pi Camera Module 2. This package uses native ROS2 camera drivers (v4l2_camera) for efficient image capture and distributed processing between Raspberry Pi and a laptop.

## 🏗️ Architecture

```
┌─────────────────────────┐         Network         ┌──────────────────────────────────┐
│   RASPBERRY PI 4        │◄────────────────────────►│          LAPTOP                  │
│                         │                          │                                  │
│  ┌──────────────────┐   │   /camera/image_raw/    │  ┌───────────────────────────┐   │
│  │ RPi Camera v2    │   │      compressed         │  │   Color Detection Node    │   │
│  │  (CSI Port)      │   │   (JPEG 80% quality)    │  │   (OpenCV HSV)            │   │
│  └────────┬─────────┘   │                          │  └───────────────────────────┘   │
│           │             │                          │                                  │
│  ┌────────▼─────────┐   │                          │  ┌───────────────────────────┐   │
│  │ v4l2_camera_node │   │                          │  │ Disease Detection Node    │   │
│  │  /dev/video0     │───┼─────────────────────────►│  │ (PyTorch ResNet18)        │   │
│  └──────────────────┘   │                          │  │  - Healthy                │   │
│                         │                          │  │  - Early Blight           │   │
└─────────────────────────┘                          │  │  - Late Blight            │   │
                                                     │  └───────────────────────────┘   │
                                                     └──────────────────────────────────┘
```

### Components

**Raspberry Pi Side:**

- 📷 **Camera Capture**: v4l2_camera_node (native ROS2 driver)
- 📦 **Image Compression**: image_transport (JPEG compression for network efficiency)
- 📡 **Publishing**: `/camera/image_raw` and `/camera/image_raw/compressed`

**Laptop Side:**

- 📥 **Image Reception**: Subscribes to compressed images over network
- 🎨 **Color Detection**: OpenCV-based HSV color space detection (red, green, blue, yellow)
- 🌱 **Disease Detection**: PyTorch ResNet18 ML model for potato leaf disease classification
- 🖼️ **Visualization**: Real-time display of detected colors and disease status
- 📊 **Processing**: All CPU-intensive tasks run on laptop (OpenCV + ML inference)

---

## 📋 Prerequisites

### Hardware

- ✅ Raspberry Pi 4 (4GB RAM)
- ✅ Raspberry Pi Camera Module 2 (connected via CSI port)
- ✅ Laptop with Ubuntu 22.04
- ✅ Network connection (WiFi or Ethernet) between Pi and Laptop

### Software Requirements

#### Both Machines

- ROS2 Humble
- Python 3.10+

#### Raspberry Pi Specific

```bash
sudo apt update
sudo apt install -y ros-humble-v4l2-camera
sudo apt install -y ros-humble-image-transport-plugins
sudo apt install -y libraspberrypi-bin
sudo apt install -y ros-humble-camera-info-manager
```

#### Laptop Specific

```bash
# ROS2 packages
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-image-transport-plugins

# Python packages
pip3 install opencv-python

# For Disease Detection (PyTorch ML model)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install pillow
pip3 install "numpy<2.0" --force-reinstall  # cv_bridge compatibility
```

---

## 🔧 Setup Instructions

### 1. Raspberry Pi Configuration

#### Enable Camera and Interfaces

```bash
sudo raspi-config
```

Navigate and enable:

- **Interface Options** → **Legacy Camera** → **Enable**
- **Interface Options** → **I2C** → **Enable**
- **Interface Options** → **SPI** → **Enable**

Reboot after changes:

```bash
sudo reboot
```

#### Verify Camera

```bash
# Check if camera device exists
ls -l /dev/video0

# List camera capabilities
v4l2-ctl --list-devices

# Check supported formats
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Expected output should show `/dev/video0` device.

#### Add User to Video Group

```bash
sudo usermod -a -G video $USER
# Log out and back in for changes to take effect
```

### 2. Network Configuration

**IMPORTANT**: Configure on **BOTH** Raspberry Pi and Laptop

Edit `~/.bashrc` and add:

```bash
# ROS2 Network Configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Apply changes:

```bash
source ~/.bashrc
```

**Verify Network Connectivity:**

```bash
# From laptop, ping Raspberry Pi
ping <raspberry_pi_ip>

# From Pi, ping laptop
ping <laptop_ip>
```

### 3. Package Installation

#### Clone and Build (Both Machines)

```bash
cd ~/ros2_ws/src

# If not already cloned
git clone <your-repo-url>

cd ~/ros2_ws
colcon build --packages-select rpi_camera_package
source install/setup.bash
```

---

## 🚀 Usage

### Quick Start

#### On Raspberry Pi:

```bash
# Terminal 1: Launch camera with compression (RECOMMENDED)
ros2 launch rpi_camera_package camera_compressed.launch.py pi

# Alternative: Raw images (higher bandwidth)
# ros2 launch rpi_camera_package camera_publisher.launch.py pi
```

#### On Laptop:

```bash
# Option 1: Color detection only
ros2 launch rpi_camera_package color_detection.launch.py laptop

# Option 2: Disease detection only (ML-based)
ros2 launch rpi_camera_package disease_detection.launch.py laptop

# Option 3: BOTH color + disease detection (parallel processing)
ros2 launch rpi_camera_package full_processing.launch.py laptop
```

> 📖 **For detailed disease detection documentation**, see [DISEASE_DETECTION.md](DISEASE_DETECTION.md)

### Verify Operation

```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list

# Check image stream frequency
ros2 topic hz /camera/image_raw/compressed

# View single compressed frame info
ros2 topic echo /camera/image_raw/compressed --once
```

---

## ⚙️ Configuration & Parameters

### Camera Parameters (Pi Side)

Edit: `config/pi/camera_params.yaml`

```yaml
/camera/camera:
  ros__parameters:
    video_device: "/dev/video0"
    image_size: [640, 480] # Resolution
    pixel_format: "YUYV" # or "MJPEG"
    time_per_frame: [1, 30] # 30 FPS
```

### Launch File Parameters

#### Camera Publisher (Pi)

```bash
# Custom video device
ros2 launch rpi_camera_package camera_publisher.launch.py video_device:=/dev/video1

# Adjust JPEG quality (0-100, higher = better quality, larger size)
ros2 launch rpi_camera_package camera_compressed.launch.py jpeg_quality:=90
```

#### Color Detection (Laptop)

```bash
# Use raw images instead of compressed
ros2 launch rpi_camera_package color_detection.launch.py use_compressed:=false

# Disable display window (headless mode)
ros2 launch rpi_camera_package color_detection.launch.py display:=false

# Adjust minimum detection area (pixels)
ros2 launch rpi_camera_package color_detection.launch.py min_area:=500

# Disable processed image publishing
ros2 launch rpi_camera_package color_detection.launch.py publish_processed:=false
```

### Color Detection Parameters

Colors detected (HSV ranges):

- 🔴 **Red**: [0, 120, 70] → [10, 255, 255]
- 🟢 **Green**: [40, 40, 40] → [70, 255, 255]
- 🔵 **Blue**: [110, 50, 50] → [130, 255, 255]
- 🟡 **Yellow**: [20, 100, 100] → [30, 255, 255]

Modify these in: `rpi_camera_package/laptop_nodes/color_detection_node.py`

---

## 📊 Topics

### Published by Raspberry Pi:

- `/camera/image_raw` (sensor_msgs/Image) - Raw camera feed
- `/camera/image_raw/compressed` (sensor_msgs/CompressedImage) - JPEG compressed feed
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera calibration info

### Published by Laptop (Color Detection):

- `/color_detection/processed_image` (sensor_msgs/Image) - Image with color detection overlays

### Published by Laptop (Disease Detection):

- `/disease_detection/result` (std_msgs/String) - Classification result (e.g., "Disease: Healthy | Confidence: 98.5%")
- `/disease_detection/annotated_image` (sensor_msgs/Image) - Image with disease status overlay

---

## 🐛 Troubleshooting

### Camera Not Detected

**Problem**: `/dev/video0` doesn't exist

**Solutions**:

```bash
# 1. Check if legacy camera is enabled
sudo raspi-config
# Interface Options → Legacy Camera → Enable

# 2. Check camera connection
vcgencmd get_camera
# Should show: supported=1 detected=1

# 3. Reboot
sudo reboot

# 4. Check dmesg for errors
dmesg | grep -i camera
```

### No Images Received on Laptop

**Problem**: Laptop node starts but receives no images

**Solutions**:

```bash
# 1. Verify Pi is publishing
# On Pi:
ros2 topic hz /camera/image_raw/compressed

# 2. Check ROS2 discovery
# On both machines:
ros2 daemon stop
ros2 daemon start

# 3. Verify network configuration
# On both machines:
echo $ROS_DOMAIN_ID  # Should be 42
echo $ROS_LOCALHOST_ONLY  # Should be 0

# 4. Check firewall
# On both machines:
sudo ufw status
# If active, allow ROS2 ports:
sudo ufw allow 7400:7600/tcp
sudo ufw allow 7400:7600/udp

# 5. Test basic connectivity
ping <other_machine_ip>
```

### Low Frame Rate

**Problem**: Frame rate is lower than expected

**Solutions**:

```bash
# 1. Use compressed images (much more efficient)
ros2 launch rpi_camera_package camera_compressed.launch.py

# 2. Lower JPEG quality for smaller size
ros2 launch rpi_camera_package camera_compressed.launch.py jpeg_quality:=60

# 3. Reduce resolution
# Edit: config/pi/camera_params.yaml
# Change image_size: [320, 240]

# 4. Check network bandwidth
# On Pi:
iftop  # Monitor network usage
```

### Permission Denied on /dev/video0

**Problem**: Cannot access camera device

**Solution**:

```bash
# Add user to video group
sudo usermod -a -G video $USER

# Apply immediately (or logout/login)
newgrp video

# Verify permissions
ls -l /dev/video0
# Should show: crw-rw---- 1 root video ...
```

### OpenCV Display Error

**Problem**: `cv2.imshow()` fails with display error

**Solution**:

```bash
# 1. Make sure DISPLAY is set
echo $DISPLAY

# 2. If running over SSH, use X forwarding
ssh -X user@laptop

# 3. Or disable display in headless mode
ros2 launch rpi_camera_package color_detection.launch.py display:=false
```

---

## 📁 Package Structure

```
rpi_camera_package/
├── config/
│   ├── pi/
│   │   └── camera_params.yaml               # Camera settings for v4l2
│   └── laptop/
│       └── disease_detection_params.yaml    # ML model configuration
├── launch/
│   ├── pi/
│   │   ├── camera_publisher.launch.py       # Raw images
│   │   └── camera_compressed.launch.py      # Compressed images (recommended)
│   └── laptop/
│       ├── color_detection.launch.py        # Color detection node
│       ├── disease_detection.launch.py      # Disease detection node (ML)
│       └── full_processing.launch.py        # Both color + disease detection
├── rpi_camera_package/
│   ├── __init__.py
│   ├── laptop_nodes/
│   │   ├── __init__.py
│   │   ├── color_detection_node.py          # OpenCV color detection
│   │   └── disease_detection_node.py        # PyTorch ML inference
│   └── common/
│       ├── __init__.py
│       └── inference_engine.py              # PyTorch model wrapper
├── models/
│   └── model_ft_gpu.pth                     # Pre-trained ResNet18 model (43MB)
├── rviz/
│   └── (future visualization configs)
├── package.xml
├── setup.py
├── README.md                                # Main documentation
├── DISEASE_DETECTION.md                     # ML model usage guide
├── QUICKSTART.md                            # Quick reference
└── TESTING_CHECKLIST.md                     # Verification steps
```

---

## 🔬 Testing

### Test Camera on Pi

```bash
# Launch camera
ros2 launch rpi_camera_package camera_compressed.launch.py pi

# In another terminal, check publishing rate
ros2 topic hz /camera/image_raw/compressed

# View compressed image data
ros2 topic echo /camera/image_raw/compressed --once
```

### Test Color Detection on Laptop

```bash
# Make sure Pi is publishing images first!

# Launch color detection
ros2 launch rpi_camera_package color_detection.launch.py laptop

# Show an object with red, blue, green, or yellow color to the camera
# Detection results will appear in terminal and display window
```

### Test Disease Detection on Laptop

```bash
# Make sure Pi is publishing images first!

# Launch disease detection
ros2 launch rpi_camera_package disease_detection.launch.py laptop

# Monitor results
ros2 topic echo /disease_detection/result

# Show potato leaves to camera:
#   - Healthy leaves → Should detect "Healthy"
#   - Diseased leaves → Should detect "Early_blight" or "Late_blight"
```

### Test Network Latency

```bash
# On laptop, measure latency
ros2 topic delay /camera/image_raw/compressed
```

---

## 🎯 Performance Tips

1. **Use Compressed Images**: Always use `camera_compressed.launch.py` on Pi for network transmission
2. **Adjust JPEG Quality**: Balance quality vs bandwidth (60-80 is usually good)
3. **Reduce Resolution**: Lower resolution = faster processing and lower bandwidth
4. **Network Quality**: Use 5GHz WiFi or Ethernet for best results
5. **QoS Settings**: Uses BEST_EFFORT reliability for real-time performance

---

## 📝 Notes

- **No Custom Camera Code**: Uses native ROS2 `v4l2_camera` driver (more reliable)
- **Network Optimized**: Compressed image transport reduces bandwidth significantly (~70-80% reduction)
- **Distributed Processing**: All heavy processing (OpenCV + ML) runs on laptop, not Pi
- **Parallel Processing**: Can run color detection and disease detection simultaneously
- **ML Model**: PyTorch ResNet18 pre-trained on potato disease dataset (3 classes)
- **Rate Limiting**: Disease detection uses configurable inference rate to manage CPU usage
- **Frame Rate**: Typical 20-30 FPS camera feed, 1 Hz ML inference (configurable)

---

## 🤝 Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

---

## 📄 License

Apache 2.0

---

## 👤 Author

**Allen Kizito Wachio**

- Email: allenkizitowachio@gmail.com
- Project: Robotics Dojo 2025

---

## 🔗 Related Packages

- [v4l2_camera](https://github.com/ros-drivers/v4l2_camera) - ROS2 V4L2 Camera Driver
- [image_transport](https://github.com/ros-perception/image_common) - Image Transport Plugins
- [cv_bridge](https://github.com/ros-perception/vision_opencv) - OpenCV Bridge

---

**Last Updated**: October 5, 2025
