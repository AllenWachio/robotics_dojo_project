# Raspberry Pi Camera Setup Guide

This guide helps you set up the camera on your Raspberry Pi for the RPI Camera Package.

## Prerequisites

- Raspberry Pi 4 (or compatible)
- Raspberry Pi Camera Module 2 (or v1.3)
- ROS2 Humble installed
- Camera connected to CSI port

## Installation Steps

### 1. Install rpicam-apps (Modern libcamera)

```bash
sudo apt-get update
sudo apt-get install rpicam-apps
```

### 2. Verify Camera Detection

```bash
# Check if camera is detected
vcgencmd get_camera

# Expected output: "supported=1 detected=1"
```

### 3. Test Camera Manually

```bash
# Take a test photo
rpicam-still -o test.jpg

# Record a 5-second test video
rpicam-vid -t 5000 -o test.h264
```

If these commands work, your camera is set up correctly!

### 4. Build the ROS2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select rpi_camera_package --symlink-install
source install/setup.bash
```

### 5. Configure Network

Add these to your `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Then source it:

```bash
source ~/.bashrc
```

## Running the Camera

Simply run the shell script:

```bash
cd ~/ros2_ws
./src/robotics_dojo_project/rpi_camera_package/scripts/pi/run_camera.sh
```

The script will:

- Check all prerequisites
- Configure the camera
- Start publishing compressed images to `/camera/image_raw/compressed`

## Troubleshooting

### Camera Not Detected

```bash
# Check camera connection
vcgencmd get_camera

# If not detected, try:
sudo raspi-config
# Navigate to: Interface Options → Legacy Camera → Disable (use libcamera)
# Reboot after changes
```

### rpicam-apps Not Found

```bash
sudo apt-get update
sudo apt-get install rpicam-apps
```

### Permission Issues

```bash
# Add user to video group
sudo usermod -a -G video $USER

# Logout and login again for changes to take effect
```

### Camera Already in Use

```bash
# Check what's using the camera
sudo fuser -v /dev/video*

# Kill the process if needed
sudo killall rpicam-vid
```

## Camera Configuration

Edit camera parameters in: `config/pi/camera_params.yaml`

Default settings (configured in run_camera.sh):

- Resolution: 1280x720
- Frame Rate: 15 FPS
- JPEG Quality: 80%

You can adjust these in the script if needed.

## Network Verification

On the Pi, check published topics:

```bash
ros2 topic list
# Should show: /camera/image_raw/compressed
```

On the laptop (same network), verify you can see the topics:

```bash
ros2 topic list
# Should show the same topics
```

## Performance Tips

1. **Lower resolution** for faster processing: Change WIDTH and HEIGHT in run_camera.sh
2. **Reduce frame rate** for less network load: Change FRAMERATE in run_camera.sh
3. **Lower JPEG quality** for smaller images: Change JPEG_QUALITY in run_camera.sh

## Next Steps

Once camera is running on Pi, launch processing nodes on laptop:

```bash
# On laptop
./src/robotics_dojo_project/rpi_camera_package/scripts/laptop/run_color_detection.sh
# OR
./src/robotics_dojo_project/rpi_camera_package/scripts/laptop/run_disease_detection.sh
```

## Technical Details

- **Camera Driver**: rpicam-vid (libcamera)
- **Image Format**: JPEG compressed (80% quality)
- **Topic**: `/camera/image_raw/compressed`
- **Message Type**: `sensor_msgs/CompressedImage`
- **Network**: DDS with FastRTPS

The rpicam approach is more reliable than v4l2 for Pi Camera Module because:

- Native support for Pi Camera hardware
- Better performance with hardware encoding
- More stable with unicam driver
- No format compatibility issues
