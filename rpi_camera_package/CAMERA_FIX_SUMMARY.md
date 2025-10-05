# Camera Fix Summary - October 5, 2025

## Problem
The v4l2_camera was failing with "Invalid argument (22)" error on Raspberry Pi Camera Module 2 because the unicam driver outputs raw Bayer sensor data, not standard RGB/YUYV formats that v4l2 expects.

## Solution
Switched from v4l2_camera to **rpicam-vid** (modern libcamera approach), which natively supports the Raspberry Pi Camera Module 2.

## Changes Made

### 1. New Camera Node
- **File**: `rpi_camera_package/rpicam_node.py`
- Uses `rpicam-vid` subprocess to capture frames
- Converts YUV420 to BGR using OpenCV
- Publishes compressed JPEG images (same format as before)
- **Topics**: Same as v4l2 approach
  - `/camera/image_raw/compressed` (sensor_msgs/CompressedImage)
  - `/camera/camera_info` (sensor_msgs/CameraInfo)

### 2. New Launch File
- **File**: `launch/pi/camera_rpicam.launch.py`
- Parameters:
  - width (default: 1280)
  - height (default: 720)
  - framerate (default: 15)
  - jpeg_quality (default: 80)

### 3. Updated Shell Script
- **File**: `scripts/pi/run_camera.sh`
- Now uses `camera_rpicam.launch.py` instead of `camera_compressed.launch.py`
- Checks for rpicam-vid installation
- Pre-configured with optimal settings:
  - 1280x720 resolution
  - 15 FPS
  - 80% JPEG quality

### 4. Updated setup.py
- Added entry point: `rpicam_node = rpi_camera_package.rpicam_node:main`

### 5. Documentation
- **File**: `PI_CAMERA_SETUP.md` - Complete setup guide for Pi
- **File**: `scripts/pi/diagnose_camera.sh` - Diagnostic tool to check camera formats

## Installation on Raspberry Pi

```bash
# Install rpicam-apps
sudo apt-get update
sudo apt-get install rpicam-apps

# Build the package
cd ~/ros2_ws
colcon build --packages-select rpi_camera_package --symlink-install
source install/setup.bash

# Run the camera
./src/robotics_dojo_project/rpi_camera_package/scripts/pi/run_camera.sh
```

## Compatibility

✅ **Laptop nodes unchanged** - Color detection and disease detection work exactly the same
✅ **Same topics** - `/camera/image_raw/compressed` published identically
✅ **Same message types** - `sensor_msgs/CompressedImage`
✅ **Same network setup** - ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=0

## Benefits of rpicam Approach

1. **Native Pi Camera support** - No format compatibility issues
2. **Better performance** - Hardware-accelerated encoding
3. **More reliable** - Works out-of-the-box with Pi Camera Module 2
4. **Maintained** - Modern libcamera is actively developed
5. **Flexible** - Easy to adjust resolution, framerate, quality

## Troubleshooting

If camera still doesn't work:
1. Run diagnostic script: `./scripts/pi/diagnose_camera.sh`
2. Test rpicam manually: `rpicam-vid -t 5000 -o test.h264`
3. Check camera detection: `vcgencmd get_camera`
4. Ensure legacy camera is disabled in `raspi-config`

## Old Approach (Deprecated)

The v4l2_camera approach is still available but not recommended:
- **Launch file**: `launch/pi/camera_compressed.launch.py`
- **Config**: `config/pi/camera_params.yaml`

To use it, you'd need to find a compatible pixel format, which varies by Pi model and kernel version.

## Verification

Camera is working correctly when:
1. Script shows "Camera started successfully"
2. On laptop: `ros2 topic list` shows `/camera/image_raw/compressed`
3. On laptop: `ros2 topic hz /camera/image_raw/compressed` shows ~15 Hz
4. Laptop nodes can process frames without errors
