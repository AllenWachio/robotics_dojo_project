# Shell Scripts - Quick Launch Guide

This directory contains self-contained shell scripts that launch different components of the RPI Camera Package with all necessary parameters pre-configured.

## 📂 Directory Structure

```
scripts/
├── pi/                           # Scripts for Raspberry Pi
│   └── run_camera.sh            # Camera launcher
└── laptop/                       # Scripts for Laptop
    ├── run_color_detection.sh   # Color detection
    ├── run_disease_detection.sh # Disease detection (ML)
    └── run_full_processing.sh   # Both detections
```

> 🎯 **Clear separation**: Pi scripts in `pi/`, Laptop scripts in `laptop/`

---

## 🚀 Available Scripts

### 1. **pi/run_camera.sh** (Raspberry Pi)

Launch camera node on Pi with compressed image transport.

**Usage:**

```bash
./run_camera.sh
```

**Pre-configured Parameters:**

- Video device: `/dev/video0`
- JPEG quality: 80%
- Launch file: `camera_compressed.launch.py`

**Checks:**

- ✅ Camera device exists
- ✅ User in video group
- ✅ ROS2 network configuration
- ✅ Workspace built and sourced

---

### 2. **laptop/run_color_detection.sh** (Laptop)

Launch color detection node on laptop.

**Usage:**

```bash
./run_color_detection.sh
```

**Pre-configured Parameters:**

- Use compressed images: `true`
- Display output: `true`
- Minimum detection area: `1000` pixels
- Publish processed images: `true`

**Detects:** 🔴 Red, 🟢 Green, 🔵 Blue, 🟡 Yellow

**Checks:**

- ✅ OpenCV installed
- ✅ ROS2 network configuration
- ✅ Camera topics available
- ✅ Workspace built and sourced

---

### 3. **laptop/run_disease_detection.sh** (Laptop)

Launch disease detection node with ML model on laptop.

**Usage:**

```bash
./run_disease_detection.sh
```

**Pre-configured Parameters:**

- Use compressed images: `true`
- Display output: `true`
- Inference rate: `1.0` Hz
- Confidence threshold: `0.0`
- Publish annotated images: `true`

**Detects:** 🟢 Healthy, 🟠 Early Blight, 🔴 Late Blight

**Checks:**

- ✅ PyTorch installed
- ✅ PIL (Pillow) installed
- ✅ ML model file exists
- ✅ ROS2 network configuration
- ✅ Camera topics available
- ✅ Workspace built and sourced

---

### 4. **laptop/run_full_processing.sh** (Laptop)

Launch BOTH color and disease detection simultaneously.

**Usage:**

```bash
./run_full_processing.sh
```

**Pre-configured Parameters:**

- Use compressed images: `true`
- Display output: `true` (2 windows)

**Runs:**

- Color detection node
- Disease detection node

**Checks:**

- ✅ All dependencies (PyTorch, OpenCV, PIL)
- ✅ ML model file exists
- ✅ System resources (CPU, memory)
- ✅ ROS2 network configuration
- ✅ Camera topics available
- ✅ Workspace built and sourced

---

## 📍 Running the Scripts

### **Option 1: From Source Directory (Recommended)**

**On Raspberry Pi:**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/pi
./run_camera.sh
```

**On Laptop:**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
./run_color_detection.sh           # Color detection only
./run_disease_detection.sh         # Disease detection only
./run_full_processing.sh           # Both at once
```

> 🎯 **Clear & Simple**: Navigate to `pi/` or `laptop/` folder and run the script!

---

### **Option 2: Direct Path (After Installation)**

**On Raspberry Pi:**

```bash
~/ros2_ws/install/rpi_camera_package/lib/rpi_camera_package/pi/run_camera.sh
```

**On Laptop:**

```bash
~/ros2_ws/install/rpi_camera_package/lib/rpi_camera_package/laptop/run_disease_detection.sh
~/ros2_ws/install/rpi_camera_package/lib/rpi_camera_package/laptop/run_color_detection.sh
~/ros2_ws/install/rpi_camera_package/lib/rpi_camera_package/laptop/run_full_processing.sh
```

---

## ✅ Pre-flight Checks

Each script performs comprehensive checks before launching:

### **All Scripts:**

- ✅ Workspace exists and is built
- ✅ ROS2 installed and in PATH
- ✅ Network configuration (ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=0)
- ✅ Sources workspace automatically

### **Camera Script (Pi):**

- ✅ Camera device exists (`/dev/video0`)
- ✅ User in video group
- ✅ Camera permissions

### **Color Detection Script:**

- ✅ OpenCV installed
- ✅ Camera topics available

### **Disease Detection Script:**

- ✅ PyTorch installed
- ✅ PIL (Pillow) installed
- ✅ ML model file exists
- ✅ Camera topics available

### **Full Processing Script:**

- ✅ All above checks combined
- ✅ System resources check (CPU, memory)

---

## 🎨 Color-Coded Output

Scripts use colored output for easy reading:

- 🟢 **Green (✅)**: Success messages
- 🔴 **Red (❌)**: Error messages
- 🟡 **Yellow (⚠️)**: Warning messages
- 🔵 **Blue (ℹ️)**: Info messages
- 🔷 **Cyan**: Section headers
- 🟣 **Magenta**: Special headers (full processing)

---

## 🐛 Troubleshooting

### **Script Won't Execute**

```bash
# Make sure scripts are executable
chmod +x ~/ros2_ws/src/rpi_camera_package/scripts/pi/*.sh
chmod +x ~/ros2_ws/src/rpi_camera_package/scripts/laptop/*.sh

# Or rebuild package
cd ~/ros2_ws
colcon build --packages-select rpi_camera_package
```

### **"Workspace not found"**

```bash
# Build the workspace first
cd ~/ros2_ws
colcon build --packages-select rpi_camera_package
```

### **"Camera device not found" (Pi)**

```bash
# Enable legacy camera
sudo raspi-config
# Interface Options → Legacy Camera → Enable

# Reboot
sudo reboot
```

### **"PyTorch not found" (Laptop)**

```bash
# Install PyTorch
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### **"Camera topics not found" (Laptop)**

```bash
# Make sure camera is running on Pi first!
# On Pi:
./run_camera.sh

# Then on Laptop:
./run_disease_detection.sh
```

---

## 🎯 Typical Workflow

### **Basic Setup:**

1. **On Pi**:

   ```bash
   cd ~/ros2_ws/src/rpi_camera_package/scripts/pi
   ./run_camera.sh
   ```

2. **On Laptop**:
   ```bash
   cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
   ./run_disease_detection.sh    # OR ./run_color_detection.sh
   ```

### **Full Analysis:**

1. **On Pi**:

   ```bash
   cd ~/ros2_ws/src/rpi_camera_package/scripts/pi
   ./run_camera.sh
   ```

2. **On Laptop**:
   ```bash
   cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
   ./run_full_processing.sh
   ```

---

## 📊 What to Expect

### **Camera Script (Pi):**

- Starts v4l2_camera_node
- Publishes to `/camera/image_raw` and `/camera/image_raw/compressed`
- Typical output: 20-30 FPS

### **Color Detection Script (Laptop):**

- Opens OpenCV window
- Shows detected colors in terminal
- Draws bounding boxes around colored objects

### **Disease Detection Script (Laptop):**

- Opens OpenCV window with disease status overlay
- Publishes classification results to `/disease_detection/result`
- Shows: "Disease: Healthy | Confidence: 98.5%"

### **Full Processing Script (Laptop):**

- Opens TWO OpenCV windows
- Runs both detection systems in parallel
- Higher CPU usage (~60-80%)

---

## 🔧 Modifying Parameters

If you need to change parameters, edit the configuration section at the top of each script:

```bash
# Example: run_disease_detection.sh
# Configuration
WORKSPACE_PATH="$HOME/ros2_ws"
USE_COMPRESSED=true
DISPLAY_OUTPUT=true
INFERENCE_RATE=1.0              # ← Change this to 0.5 for slower inference
CONFIDENCE_THRESHOLD=0.0        # ← Change this to 0.8 for higher confidence
PUBLISH_ANNOTATED=true
```

---

## 📚 Related Documentation

- [README.md](../README.md) - Main package documentation
- [QUICKSTART.md](../QUICKSTART.md) - Quick reference guide
- [DISEASE_DETECTION.md](../DISEASE_DETECTION.md) - ML model usage guide
- [QUICK_REFERENCE.md](../QUICK_REFERENCE.md) - Fast command reference

---

**Note:** These scripts are designed for beginners. They include extensive checks and helpful error messages to guide you through any issues.

---

**Last Updated**: October 5, 2025
