# Quick Setup Guide - RPI Camera Package

## 🚀 Easiest Way: Shell Scripts (Recommended!)

### **On Raspberry Pi:**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/pi
./run_camera.sh
```

### **On Laptop:**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop

# Option 1: Color detection only
./run_color_detection.sh

# Option 2: Disease detection only (ML-based)
./run_disease_detection.sh

# Option 3: BOTH color + disease detection
./run_full_processing.sh
```

> ✅ **Scripts include all pre-flight checks, error handling, and helpful messages!**
> 📖 See `scripts/README.md` for full documentation

---

## 🚀 Alternative: Direct Launch Commands

### On Raspberry Pi (Terminal 1):

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch camera with compression (RECOMMENDED for network)
ros2 launch rpi_camera_package camera_compressed.launch.py pi
```

### On Laptop (Terminal 1):

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Option 1: Color detection only
ros2 launch rpi_camera_package color_detection.launch.py laptop

# Option 2: Disease detection only (ML-based)
ros2 launch rpi_camera_package disease_detection.launch.py laptop

# Option 3: BOTH color + disease detection
ros2 launch rpi_camera_package full_processing.launch.py laptop
```

---

## ✅ Verification Steps

### 1. Check Network Configuration (Both Machines)

```bash
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"           # Should be 42
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY" # Should be 0
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"  # Should be rmw_fastrtps_cpp
```

### 2. Verify Camera on Pi

```bash
# Check device
ls -l /dev/video0

# Test v4l2
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### 3. Test Topics (After launching nodes)

```bash
# List all topics
ros2 topic list

# Check camera publishing rate
ros2 topic hz /camera/image_raw/compressed

# Check nodes
ros2 node list
```

---

## 🔧 Common Commands

### Pi Side

```bash
# Launch with custom video device
ros2 launch rpi_camera_package camera_publisher.launch.py pi video_device:=/dev/video1

# Adjust JPEG quality (60-100)
ros2 launch rpi_camera_package camera_compressed.launch.py pi jpeg_quality:=70
```

### Laptop Side - Color Detection

```bash
# Use raw images (not compressed)
ros2 launch rpi_camera_package color_detection.launch.py laptop use_compressed:=false

# Run without display window (headless)
ros2 launch rpi_camera_package color_detection.launch.py laptop display:=false

# Adjust minimum detection area
ros2 launch rpi_camera_package color_detection.launch.py laptop min_area:=500
```

### Laptop Side - Disease Detection

```bash
# Adjust inference rate (Hz)
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=1.0

# Set confidence threshold (0.0-1.0)
ros2 launch rpi_camera_package disease_detection.launch.py laptop confidence_threshold:=0.5

# Disable display (headless)
ros2 launch rpi_camera_package disease_detection.launch.py laptop display:=false

# Monitor results
ros2 topic echo /disease_detection/result
```

---

## 🐛 Quick Troubleshooting

### No Images on Laptop?

```bash
# On Pi - verify publishing
ros2 topic hz /camera/image_raw/compressed

# On both - restart ROS2 daemon
ros2 daemon stop && ros2 daemon start

# Check network
ping <other_machine_ip>
```

### Camera Not Found?

```bash
# Enable legacy camera
sudo raspi-config
# Interface Options → Legacy Camera → Enable

# Add user to video group
sudo usermod -a -G video $USER
newgrp video

# Reboot
sudo reboot
```

### Permission Denied?

```bash
# Check permissions
ls -l /dev/video0

# Fix if needed
sudo chmod 666 /dev/video0
```

---

## 📦 Package Structure

```
Pi:     Uses v4l2_camera_node (native ROS2 driver)
Laptop: color_detection_node (OpenCV processing)
        disease_detection_node (PyTorch ML inference)
```

### Installed Executables:

- `color_detection_node` - OpenCV color detection
- `disease_detection_node` - ML disease classification

### Launch Files (Pi):

- `launch/pi/camera_publisher.launch.py` - Raw images
- `launch/pi/camera_compressed.launch.py` - Compressed images ⭐

### Launch Files (Laptop):

- `launch/laptop/color_detection.launch.py` - Color detection only
- `launch/laptop/disease_detection.launch.py` - Disease detection only ⭐
- `launch/laptop/full_processing.launch.py` - Both color + disease ⭐

---

## 📊 Expected Performance

### Camera Feed

- **Frame Rate**: 20-30 FPS over WiFi with compression
- **Latency**: 50-150ms (depends on network)
- **Bandwidth**: ~1-3 Mbps with JPEG compression (quality 80)
- **CPU (Pi)**: ~15-25% for v4l2 + compression

### Laptop Processing

- **Color Detection**: ~30-50% CPU + display
- **Disease Detection**: ~30-50% CPU (1 Hz inference) + display
- **Both Together**: ~60-80% CPU total
- **ML Inference Time**: 100-300ms per frame (CPU), 20-50ms (GPU)

---

## 🎯 Test Scenarios

### Color Detection Test

1. **Start Pi camera**
2. **Wait 5 seconds**
3. **Start Laptop color detection**
4. **Show colored objects** (red, blue, green, yellow)
5. **Check terminal** for detection messages
6. **View window** to see bounding boxes

### Disease Detection Test

1. **Start Pi camera**
2. **Wait 5 seconds**
3. **Start Laptop disease detection**
4. **Show potato leaves** to camera
   - Healthy leaves → Should detect "Healthy"
   - Diseased leaves → Should detect "Early_blight" or "Late_blight"
5. **Check terminal** for classification results with confidence
6. **View window** to see disease status overlay

---

## 📚 Documentation

- [README.md](README.md) - Main documentation
- [DISEASE_DETECTION.md](DISEASE_DETECTION.md) - ML model usage guide ⭐
- [TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) - Verification steps
