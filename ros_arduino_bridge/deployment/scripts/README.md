# ROS2 Arduino Bridge - Distributed Computing Scripts

This directory contains organized scripts for running the distributed SLAM and navigation system with **separated hardware control** for better reliability.

## 🏗️ Architecture Overview

- **Raspberry Pi**: Runs hardware interface (Arduino + LiDAR in separate terminals)
- **Laptop**: Runs processing-intensive tasks (SLAM, navigation, visualization)
- **Separated Approach**: Arduino and LiDAR launched independently to avoid conflicts

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

**Step 1 - Start Pi Hardware (on Raspberry Pi - 2 terminals needed):**

**Terminal 1 (Arduino):**

```bash
./01_arduino_only.sh
```

**Terminal 2 (LiDAR):**

```bash
./02_lidar_only.sh
# Choose option 4 (minimal config) if other options timeout
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
# Enter a unique name for your map when prompted
# Script will prevent overwriting existing maps
```

### 3. Navigation Mode (Autonomous Navigation)

**Step 1 - Start Pi Hardware (on Raspberry Pi - 2 terminals needed):**

**Terminal 1 (Arduino):**

```bash
./01_arduino_only.sh
```

**Terminal 2 (LiDAR):**

```bash
./02_lidar_only.sh
# Use option 4 if you had timeout issues during mapping
```

**Step 2 - Start Laptop Navigation (on Laptop):**

```bash
./02_navigation_mode.sh [map_name]
# OR just ./02_navigation_mode.sh and choose from available maps
```

**Step 3 - Set Navigation Goals:**

- Use RViz "2D Pose Estimate" to set robot's initial position
- Use RViz "Nav2 Goal" to send navigation goals

## 📁 File Structure

```
deployment/
├── scripts/                     # Organized execution scripts
│   ├── README.md               # This file - UPDATED
│   ├── laptop/                 # Scripts for laptop
│   │   ├── 01_mapping_mode.sh  # Start SLAM mapping
│   │   ├── 02_navigation_mode.sh # Start autonomous navigation (enhanced)
│   │   ├── 03_save_map.sh     # Save map with user naming (enhanced)
│   │   └── 04_diagnostics.sh  # System diagnostics
│   └── pi/                     # Scripts for Raspberry Pi
│       ├── 01_arduino_only.sh  # Arduino bridge only (NEW)
│       ├── 02_lidar_only.sh    # LiDAR with test options (NEW)
│       ├── 02_setup_test.sh    # Test hardware setup
│       ├── 03_network_config.sh # Network configuration
│       ├── fix_usb_permissions.sh # USB permission fixes (NEW)
│       └── old_scripts/        # Backup of replaced scripts
├── laptop/                     # Reorganized laptop files
│   ├── launch/                 # All .launch.py files
│   └── config/                 # All .yaml and .rviz files
├── pi/                         # Reorganized pi files
│   ├── launch/                 # All .launch.py files
│   └── config/                 # Pi configuration files
└── shared/                     # Shared robot description files
```

## 🔧 Troubleshooting

### LiDAR Issues (Most Common)

**If LiDAR fails with timeout errors:**

1. **Try different configurations**: `./02_lidar_only.sh` → Option 4 (minimal config)
2. **Fix permissions**: `./fix_usb_permissions.sh`
3. **Check power supply**: Low voltage (< 1.1V) causes timeouts
4. **Try different USB port**: LiDAR and Arduino might swap ports

### Other Common Issues

1. **Build Errors**: Run `./04_diagnostics.sh` to check workspace build
2. **USB Device Not Found**: Run `./fix_usb_permissions.sh` and check connections
3. **Network Issues**: Ensure both systems are on same network with same ROS_DOMAIN_ID
4. **Map Not Found**: Maps save to `~/ros2_ws/maps/` - check this directory

### 🍓 Raspberry Pi Setup

**File structure on Pi:**

```
~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/deployment/
```

**Transfer to Pi:**

```bash
# Copy entire deployment folder to Pi
scp -r ~/ros2_ws/src/ros_arduino_bridge/deployment/ pi@<pi_ip>:~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/
```

**First-time Pi setup:**

```bash
cd ~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/deployment/scripts/pi
chmod +x *.sh
./fix_usb_permissions.sh    # Fix USB access
./02_setup_test.sh          # Test hardware
./03_network_config.sh      # Configure network
```

### Network Configuration

Both systems should have the same environment settings:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Add these to your ~/.bashrc file for automatic setup.

## 🔌 Hardware Requirements

- **Arduino**: Connected to Pi via USB (typically /dev/ttyUSB0 or ttyUSB1)
- **LiDAR**: SLLiDAR compatible device via USB (auto-detected)
- **Power**: Stable 5V supply (voltage issues cause LiDAR timeouts)
- **Network**: Both Pi and laptop on same WiFi network
- **Robot**: Calibrated for 85mm wheels, 20.8cm wheelbase

## 🚀 Next Steps

1. **Test separated hardware approach**: Arduino + LiDAR in separate terminals
2. **Create and save multiple maps**: Each with unique names
3. **Test autonomous navigation**: Load saved maps by name
4. **Fine-tune LiDAR**: Use option 4 if standard configs fail
5. **Adjust parameters**: Config files now organized in config/ directories

## 📖 Additional Documentation

- `COMPLETE_WORKFLOW.md` - Detailed step-by-step workflow
- `pi/PI_SCRIPTS_ORGANIZED.md` - Pi script organization guide
- `../REORGANIZATION_COMPLETE.md` - File structure changes
- `pi/cleanup_old_scripts.sh` - Shows current vs old scripts

## ⚡ Quick Reference

**Pi Workflow (2 terminals):**

```bash
Terminal 1: ./01_arduino_only.sh
Terminal 2: ./02_lidar_only.sh (option 4 if issues)
```

**Laptop Workflow:**

```bash
Mapping:    ./01_mapping_mode.sh → ./03_save_map.sh
Navigation: ./02_navigation_mode.sh [map_name]
```

**Troubleshooting:**

```bash
./fix_usb_permissions.sh     # USB issues
./02_setup_test.sh           # Hardware test
./04_diagnostics.sh          # System check
```
