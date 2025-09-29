# ROS2 Arduino Bridge - Distributed Computing Scripts

This directory contains organized scripts for running the distributed SLAM and navigation system.

## Architecture Overview

- **Raspberry Pi**: Runs hardware interface (Arduino communication, LiDAR)
- **Laptop**: Runs processing-intensive tasks (SLAM, navigation, visualization)

## Quick Start Guide

### 1. Initial Setup

**On Raspberry Pi:**

```bash
cd ~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/deployment/scripts/pi
chmod +x *.sh
./02_setup_test.sh  # Check hardware and setup
./03_network_config.sh  # Configure networking
```

**On Laptop:**

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
chmod +x *.sh
./04_diagnostics.sh  # Check system status
```

### 2. Mapping Mode (Create a Map)

**Step 1 - Start Pi Hardware (on Raspberry Pi):**

```bash
./01_hardware_interface.sh
```

**Step 2 - Start Laptop Mapping (on Laptop):**

```bash
./01_mapping_mode.sh
```

**Step 3 - Drive the Robot:**

- Use teleop in RViz or keyboard teleop to drive the robot
- Explore the room thoroughly to build a complete map

**Step 4 - Save the Map (on Laptop):**

```bash
./03_save_map.sh
```

### 3. Navigation Mode (Autonomous Navigation)

**Step 1 - Start Pi Hardware (on Raspberry Pi):**

```bash
./01_hardware_interface.sh
```

**Step 2 - Start Laptop Navigation (on Laptop):**

```bash
./02_navigation_mode.sh
```

**Step 3 - Set Navigation Goals:**

- Use RViz "2D Pose Estimate" to set robot's initial position
- Use RViz "Nav2 Goal" to send navigation goals

## File Structure

```
deployment/
├── scripts/                     # Organized execution scripts
│   ├── README.md               # This file
│   ├── laptop/                 # Scripts for laptop
│   │   ├── 01_mapping_mode.sh  # Start SLAM mapping
│   │   ├── 02_navigation_mode.sh # Start autonomous navigation
│   │   ├── 03_save_map.sh     # Save map after mapping
│   │   └── 04_diagnostics.sh  # System diagnostics
│   └── pi/                     # Scripts for Raspberry Pi
│       ├── 01_hardware_interface.sh # Start hardware nodes
│       ├── 02_setup_test.sh    # Test hardware setup
│       └── 03_network_config.sh # Network configuration
├── laptop/                     # Launch files and configs for laptop
├── pi/                         # Launch files for Raspberry Pi
└── shared/                     # Shared robot description files
```

## Troubleshooting

### Common Issues

1. **Build Errors**: Run diagnostics script to check workspace build
2. **USB Device Not Found**: Check device connections and permissions
3. **Network Issues**: Ensure both systems are on same network with same ROS_DOMAIN_ID
4. **Launch File Errors**: Make sure workspace is built with `colcon build`

### Pi-Specific Path

The Raspberry Pi file structure is:

```
~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/deployment/
```

**To transfer to Pi:**

```bash
# Copy the entire deployment folder to your Pi
scp -r ~/ros2_ws/src/ros_arduino_bridge/deployment/ pi@<pi_ip>:~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/
```

### Network Configuration

Both systems should have the same environment settings:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Add these to your ~/.bashrc file for automatic setup.

## Hardware Requirements

- **Arduino**: Connected to Pi via USB (typically /dev/ttyUSB0)
- **LiDAR**: Connected to Pi via USB (typically /dev/ttyUSB1)
- **Network**: Both Pi and laptop on same network
- **Robot**: Calibrated for 85mm wheels, 20.8cm wheelbase

## Next Steps

1. Test the complete system with mapping
2. Create and save a room map
3. Test autonomous navigation
4. Adjust parameters as needed in config files
