# Shell Scripts Implementation Summary

**Date**: October 5, 2025  
**Package**: rpi_camera_package  
**Feature**: Self-contained launch scripts with pre-configured parameters  
**Update**: Scripts organized into `pi/` and `laptop/` subdirectories for clarity

---

## 🎯 Implementation Overview

Created 4 self-contained shell scripts that launch different components of the RPI Camera Package with all necessary parameters pre-configured. These scripts are beginner-friendly and include comprehensive pre-flight checks.

**Key Organization**: Scripts are now organized into subdirectories:

- **`scripts/pi/`** - Scripts for Raspberry Pi
- **`scripts/laptop/`** - Scripts for Laptop

This makes it crystal clear which scripts belong where!

---

## 📂 Directory Structure

```
scripts/
├── README.md                     # Script documentation
├── pi/                           # Raspberry Pi scripts
│   └── run_camera.sh            # Camera launcher
└── laptop/                       # Laptop scripts
    ├── run_color_detection.sh   # Color detection
    ├── run_disease_detection.sh # Disease detection (ML)
    └── run_full_processing.sh   # Both detections
```

**Installation Structure:**

```
~/ros2_ws/install/rpi_camera_package/lib/rpi_camera_package/
├── pi/
│   └── run_camera.sh
└── laptop/
    ├── run_color_detection.sh
    ├── run_disease_detection.sh
    └── run_full_processing.sh
```

---

## 📦 Scripts Created

### **1. run_camera.sh (Raspberry Pi)**

- **Purpose**: Launch camera node with compressed image transport
- **Pre-configured Parameters**:

  - Video device: `/dev/video0`
  - JPEG quality: `80%`
  - Launch file: `camera_compressed.launch.py`
  - Namespace: `pi`

- **Pre-flight Checks**:
  - ✅ Workspace exists and is built
  - ✅ Camera device exists (`/dev/video0`)
  - ✅ User in video group
  - ✅ Camera permissions
  - ✅ ROS2 installed
  - ✅ Network configuration (ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY)

---

### **2. run_color_detection.sh (Laptop)**

- **Purpose**: Launch color detection node with OpenCV processing
- **Pre-configured Parameters**:

  - Use compressed images: `true`
  - Display output: `true`
  - Minimum detection area: `1000` pixels
  - Publish processed images: `true`
  - Launch file: `color_detection.launch.py`
  - Namespace: `laptop`

- **Pre-flight Checks**:
  - ✅ Workspace exists and is built
  - ✅ ROS2 installed
  - ✅ OpenCV installed
  - ✅ Network configuration
  - ✅ Camera topics available (with fallback to raw if compressed missing)

---

### **3. run_disease_detection.sh (Laptop)**

- **Purpose**: Launch disease detection node with PyTorch ML model
- **Pre-configured Parameters**:

  - Use compressed images: `true`
  - Display output: `true`
  - Inference rate: `1.0` Hz
  - Confidence threshold: `0.0`
  - Publish annotated images: `true`
  - Launch file: `disease_detection.launch.py`
  - Namespace: `laptop`

- **Pre-flight Checks**:
  - ✅ Workspace exists and is built
  - ✅ ROS2 installed
  - ✅ PyTorch installed (with version display)
  - ✅ PIL (Pillow) installed
  - ✅ OpenCV installed
  - ✅ ML model file exists (43MB model_ft_gpu.pth)
  - ✅ Network configuration
  - ✅ Camera topics available

---

### **4. run_full_processing.sh (Laptop)**

- **Purpose**: Launch BOTH color and disease detection simultaneously
- **Pre-configured Parameters**:

  - Use compressed images: `true`
  - Display output: `true` (2 windows)
  - Launch file: `full_processing.launch.py`
  - Namespace: `laptop`

- **Pre-flight Checks**:
  - ✅ All checks from color detection script
  - ✅ All checks from disease detection script
  - ✅ System resources check (CPU cores, available memory)
  - ✅ Warning about expected CPU usage (60-80%)

---

## ✨ Key Features

### **1. Color-Coded Output**

All scripts use colored terminal output:

- 🟢 **Green (✅)**: Success messages
- 🔴 **Red (❌)**: Error messages with solutions
- 🟡 **Yellow (⚠️)**: Warnings
- 🔵 **Blue (ℹ️)**: Information
- 🔷 **Cyan**: Section headers
- 🟣 **Magenta**: Special headers (full processing)

### **2. Comprehensive Error Handling**

- Every check includes specific error messages
- Solutions provided for each error
- Graceful exit codes (130 for Ctrl+C)
- Temporary environment variable setup if missing

### **3. Pre-flight Validation**

Each script validates:

- Workspace structure and build status
- Required dependencies (ROS2, Python packages)
- Hardware availability (camera device)
- Network configuration
- Topic availability
- File existence (models, configs)

### **4. Automatic Setup**

- Auto-sources workspace
- Sets temporary environment variables if missing
- Provides clear instructions for permanent fixes

### **5. User-Friendly**

- Beautiful ASCII header boxes
- Progress indicators
- Helpful information about what will happen
- Clear shutdown messages

---

## 📂 File Structure (Updated)

**Source Directory:**

```
rpi_camera_package/
├── scripts/
│   ├── README.md                    # Script documentation
│   ├── pi/                          # Raspberry Pi scripts
│   │   └── run_camera.sh           # Camera launcher
│   └── laptop/                      # Laptop scripts
│       ├── run_color_detection.sh   # Color detection
│       ├── run_disease_detection.sh # Disease detection (ML)
│       └── run_full_processing.sh   # Both detections
```

**Installation Location:**

```
~/ros2_ws/install/rpi_camera_package/lib/rpi_camera_package/
├── pi/
│   └── run_camera.sh -> (symlink to source)
└── laptop/
    ├── run_color_detection.sh -> (symlink to source)
    ├── run_disease_detection.sh -> (symlink to source)
    └── run_full_processing.sh -> (symlink to source)
```

> 🎯 **Clear Separation**: Pi scripts in `pi/`, Laptop scripts in `laptop/`

---

## 🚀 Usage Examples

### **Method 1: From Source (Recommended)**

**On Raspberry Pi:**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/pi
./run_camera.sh
```

**On Laptop:**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
./run_disease_detection.sh         # Disease detection only
./run_color_detection.sh           # Color detection only
./run_full_processing.sh           # Both at once
```

### **Method 2: Direct Path (After Installation)**

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

## 🔧 Implementation Details

### **Header Format**

```bash
################################################################################
# RPI Camera Package - [Script Name]
#
# Description of what the script does
#
# Usage: ./script_name.sh
#
# Author: Allen Kizito Wachio
# Date: October 5, 2025
################################################################################
```

### **Color Code Variables**

```bash
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
```

### **Helper Functions**

- `print_header()` - Display ASCII box header
- `print_success()` - Green success message
- `print_error()` - Red error message with exit
- `print_warning()` - Yellow warning message
- `print_info()` - Blue information message

### **Check Pattern**

```bash
print_info "Checking [component]..."
if [condition]; then
    print_success "[Component] found"
else
    print_error "[Component] not found"
    echo "Solution: [how to fix]"
    exit 1
fi
```

---

## 📊 Typical Output Example

### **Successful Launch (Disease Detection)**

```
╔════════════════════════════════════════════════════════════════╗
║    RPI Camera Package - Disease Detection Launcher (Laptop)   ║
╚════════════════════════════════════════════════════════════════╝

═══ Pre-flight Checks ═══

ℹ️  Checking workspace...
✅ Workspace found
ℹ️  Checking ROS2 installation...
✅ ROS2 found
ℹ️  Checking PyTorch installation...
✅ PyTorch installed (version: 2.0.1+cpu)
ℹ️  Checking PIL installation...
✅ PIL installed
ℹ️  Checking OpenCV installation...
✅ OpenCV installed
ℹ️  Checking network configuration...
✅ ROS_DOMAIN_ID=42
✅ ROS_LOCALHOST_ONLY=0

═══ Sourcing Workspace ═══

ℹ️  Sourcing /home/user/ros2_ws/install/setup.bash...
✅ Workspace sourced

═══ Checking ML Model ═══

ℹ️  Looking for PyTorch model...
✅ Model found (size: 43M)

═══ Checking Camera Feed ═══

ℹ️  Looking for camera topics...
✅ Camera topics found
✅ Compressed image topic available

═══ Launch Configuration ═══

ℹ️  Use Compressed Images: true
ℹ️  Display Output: true
ℹ️  Inference Rate: 1.0 Hz
ℹ️  Confidence Threshold: 0.0
ℹ️  Publish Annotated Images: true
ℹ️  Model: ResNet18 (3 classes)
ℹ️  Launch File: disease_detection.launch.py
ℹ️  Namespace: laptop

═══ Starting Disease Detection Node ═══

✅ Launching disease detection with ML model...

ℹ️  Disease classes: 🟢 Healthy  🟠 Early Blight  🔴 Late Blight
ℹ️  Results published to: /disease_detection/result
ℹ️  Annotated images: /disease_detection/annotated_image

Press Ctrl+C to stop

────────────────────────────────────────────────────────────────

[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [disease_detection_node-1]: process started with pid [12345]
...
```

---

## 📚 Documentation Updates

Updated the following documentation files:

### **1. scripts/README.md (NEW)**

- Complete script documentation
- Usage instructions (3 methods)
- Pre-flight checks list
- Troubleshooting guide
- Parameter modification guide

### **2. QUICK_REFERENCE.md**

- Added "Easiest Way: Shell Scripts" section at top
- Added "Why Use Shell Scripts?" section
- Added scripts/README.md to documentation table

### **3. QUICKSTART.md**

- Reorganized with "Easiest Way: Shell Scripts" as primary method
- Moved direct launch commands to "Alternative" section
- Added link to scripts/README.md

### **4. README.md**

- Added "Quick Start (Easiest - Shell Scripts)" section
- Reorganized usage section
- Added link to scripts/README.md

---

## ✅ Benefits Over Direct Launch Commands

| Aspect                | Direct Launch          | Shell Scripts              |
| --------------------- | ---------------------- | -------------------------- |
| **Ease of Use**       | Remember long commands | Just run script            |
| **Error Prevention**  | Manual checks needed   | Automatic validation       |
| **Debugging**         | Generic ROS errors     | Specific, helpful messages |
| **Setup**             | Manual sourcing        | Auto-sources workspace     |
| **Parameters**        | Type each time         | Pre-configured             |
| **Beginner-Friendly** | ❌ No                  | ✅ Yes                     |
| **Production Ready**  | ❌ No                  | ✅ Yes                     |

---

## 🎯 Design Decisions

1. **Pre-configured Parameters**: All parameters set to optimal values for beginners
2. **Colored Output**: Visual hierarchy makes output scannable
3. **Exit Codes**: Proper codes for different scenarios (0=success, 1=error, 130=Ctrl+C)
4. **Temporary Fallbacks**: Sets environment variables temporarily if missing
5. **Non-blocking Warnings**: Camera topic checks warn but don't fail (node will wait)
6. **Helpful Error Messages**: Every error includes solution
7. **ASCII Box Headers**: Professional appearance
8. **Consistent Structure**: All scripts follow same pattern

---

## 🧪 Testing Performed

- ✅ Scripts are executable (`chmod +x`)
- ✅ Scripts installed correctly via setup.py
- ✅ Symlinks created properly in install directory
- ✅ Package builds successfully with scripts
- ✅ Script structure validated
- ⏳ Actual hardware testing pending (Pi + Laptop)

---

## 🔮 Future Enhancements

Potential improvements for future versions:

1. **Interactive Mode**: Ask user for parameters if desired
2. **Config File Support**: Load parameters from external config
3. **Logging**: Save script output to log files
4. **Status Dashboard**: Show live status of all components
5. **Auto-Recovery**: Restart on crash
6. **Multi-Camera Support**: Handle multiple cameras
7. **Remote Execution**: SSH integration for remote start

---

## 📈 Impact

### **For Beginners:**

- ✅ Can run system without understanding ROS2 launch files
- ✅ Clear error messages guide troubleshooting
- ✅ Don't need to remember complex commands
- ✅ Can focus on using the system, not configuring it

### **For Development:**

- ✅ Faster iteration (no typing long commands)
- ✅ Consistent test environment
- ✅ Easier collaboration (shared scripts)
- ✅ Professional presentation

### **For Documentation:**

- ✅ Simpler "Getting Started" section
- ✅ Reference material for error messages
- ✅ Clear entry points for users

---

## 🔗 Related Files

- [scripts/README.md](scripts/README.md) - Full script documentation
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Quick command reference
- [QUICKSTART.md](QUICKSTART.md) - Getting started guide
- [README.md](README.md) - Main package documentation

---

## ✅ Implementation Checklist

- [x] Create run_camera.sh (Pi)
- [x] Create run_color_detection.sh (Laptop)
- [x] Create run_disease_detection.sh (Laptop)
- [x] Create run_full_processing.sh (Laptop)
- [x] Make all scripts executable
- [x] Update setup.py to install scripts
- [x] Build and verify installation
- [x] Create scripts/README.md
- [x] Update QUICK_REFERENCE.md
- [x] Update QUICKSTART.md
- [x] Update main README.md
- [ ] Test on actual Pi hardware
- [ ] Test on actual Laptop hardware
- [ ] Collect user feedback

---

**Implementation Status**: ✅ **COMPLETE**

All scripts created, tested for installation, and documented. Ready for hardware validation.

---

**Author**: Allen Kizito Wachio  
**Project**: Robotics Dojo 2025  
**Date**: October 5, 2025
