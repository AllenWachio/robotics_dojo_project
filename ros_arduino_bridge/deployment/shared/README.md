# DISTRIBUTED ROBOT COMPUTING - DEPLOYMENT GUIDE

# ================================================

This directory contains organized deployment files for the distributed robot system.

## 📁 Directory Structure

```
deployment/
├── laptop/                          # Laptop base station files
│   ├── laptop_base_station.launch.py    # Main launch file
│   ├── slam_config_laptop.yaml          # SLAM parameters (full power)
│   ├── laptop_rviz_config.rviz          # RViz configuration
│   ├── setup_laptop.sh                  # Laptop setup script
│   └── saved_maps/                      # Directory for saved maps
├── pi/                              # Raspberry Pi robot files
│   ├── pi_robot_hardware.launch.py      # Hardware interface launch
│   ├── setup_pi.sh                      # Pi setup script
└── shared/                          # Files used by both systems
    ├── robot_description.xacro          # Robot URDF description
    └── README.md                        # This file
```

## 🚀 Quick Start

### 1. Setup Raspberry Pi (Robot)

```bash
# On Raspberry Pi
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/pi
./setup_pi.sh

# Test hardware
./test_hardware.sh

# Start robot services
./start_robot.sh
```

### 2. Setup Laptop (Base Station)

```bash
# On laptop
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/laptop
./setup_laptop.sh

# Test connection to robot
./test_robot_connection.sh

# Start base station
./start_base_station.sh
```

## 🔧 System Architecture

### Raspberry Pi 4 (Robot Hardware)

- **Arduino Bridge**: Odometry, motor control
- **LiDAR Driver**: Scan data publishing
- **Robot State Publisher**: TF tree
- **Topics Published**: `/odom`, `/scan`, `/tf`, `/joint_states`
- **Subscribed**: `/cmd_vel` (motor commands)

### Laptop (Base Station)

- **SLAM Processing**: Map building and localization
- **RViz Visualization**: Real-time monitoring
- **Remote Teleop**: Robot control
- **Map Management**: Save/load maps
- **Diagnostics**: System health monitoring

## 🌐 Network Requirements

- Both devices on same network (WiFi/Ethernet)
- Same `ROS_DOMAIN_ID=0`
- `ROS_LOCALHOST_ONLY=0` (network communication enabled)
- No firewall blocking ROS2 ports (7400-7500 range)

## 📊 Topic Flow

```
Raspberry Pi → Network → Laptop
/odom           ────────→ SLAM Processing
/scan           ────────→ SLAM Processing
/tf             ────────→ Visualization
/joint_states   ────────→ RViz

Laptop → Network → Raspberry Pi
/cmd_vel        ────────→ Motor Control
/map            ────────→ (optional viewing)
```

## 🛠️ Troubleshooting

### Robot Topics Not Visible

1. Check network connectivity: `ping <robot_ip>`
2. Verify ROS_DOMAIN_ID matches on both systems
3. Ensure robot services are running: `ssh pi@<robot_ip> 'ros2 node list'`

### SLAM Not Working

1. Verify `/scan` and `/odom` topics: `ros2 topic list`
2. Check topic rates: `ros2 topic hz /scan` and `ros2 topic hz /odom`
3. Verify TF tree: `ros2 run tf2_tools view_frames`

### Poor Performance

1. Use wired connection instead of WiFi
2. Reduce scan frequency on robot side
3. Check network bandwidth: `iperf3` between devices

## 📦 Adding New Features

### For Robot (Pi) Side

- Add files to `deployment/pi/`
- Hardware interfaces, sensors, actuators
- Keep processing minimal

### For Base Station (Laptop) Side

- Add files to `deployment/laptop/`
- AI processing, navigation, UI components
- Leverage full computing power

### Shared Resources

- Add common files to `deployment/shared/`
- Robot descriptions, common parameters
- Used by both systems

## 🗺️ Mapping & Navigation Workflow

### Phase 1: Room Mapping

```
1. Start robot hardware    → ./pi/setup_pi.sh
2. Start SLAM on laptop    → ./laptop/start_base_station.sh
3. Drive robot around room → Teleop keyboard
4. Save completed map      → ./laptop/save_robot_map.sh
```

### Phase 2: Autonomous Navigation

```
1. Start robot hardware    → ./pi/setup_pi.sh
2. Start navigation        → ./laptop/start_navigation.sh <map_name>
3. Set initial pose        → RViz "2D Pose Estimate" tool
4. Set navigation goals    → RViz "2D Nav Goal" tool
5. Watch autonomous nav!   → Robot navigates independently
```

## 🎯 Benefits of This Architecture

- **Scalability**: Easy to add more robots or processing nodes
- **Reliability**: Robot operates independently of laptop connection
- **Performance**: Processing power where it's needed
- **Development**: Test and debug with full laptop capabilities
- **Deployment**: Clean separation of concerns
- **Navigation**: Full autonomous navigation with obstacle avoidance
