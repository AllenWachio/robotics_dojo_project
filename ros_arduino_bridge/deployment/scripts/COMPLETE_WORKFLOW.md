# Complete Robot Workflow Guide

## New Streamlined Approach (Recommended)

### **Raspberry Pi Setup (2 separate terminals)**

#### Terminal 1: Arduino Bridge

```bash
cd ~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/deployment/scripts/pi
./01_arduino_only.sh
```

#### Terminal 2: LiDAR (after Arduino is running)

```bash
./02_lidar_only.sh
# Choose configuration option (try 1 first, then 2 if it fails)
```

### **Laptop Setup (after Pi is running)**

#### For Mapping Mode:

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./01_mapping_mode.sh
```

#### For Navigation Mode:

```bash
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
./02_navigation_mode.sh
```

#### Save Map (after mapping):

```bash
./03_save_map.sh
```

---

## Script Status & Recommendations

### **KEEP These Scripts:**

- ✅ `01_arduino_only.sh` (NEW - Arduino bridge only)
- ✅ `02_lidar_only.sh` (NEW - LiDAR with testing options)
- ✅ `fix_usb_permissions.sh` (NEW - Quick permission fix)
- ✅ `02_setup_test.sh` (Hardware testing)
- ✅ `03_network_config.sh` (Network setup)

### **CAN REMOVE (superseded by new approach):**

- ❌ `01_hardware_interface.sh` (replaced by 01_arduino_only.sh + 02_lidar_only.sh)
- ❌ `04_lidar_troubleshoot.sh` (functionality merged into 02_lidar_only.sh)

### **Laptop Scripts (unchanged):**

- ✅ `01_mapping_mode.sh`
- ✅ `02_navigation_mode.sh`
- ✅ `03_save_map.sh`
- ✅ `04_diagnostics.sh`

---

## Complete Workflow Steps

### **Phase 1: Initial Setup (One Time)**

1. **Pi Hardware Test:**

   ```bash
   ./02_setup_test.sh
   ```

2. **Network Config:**

   ```bash
   ./03_network_config.sh
   ```

3. **Fix USB Permissions:**
   ```bash
   ./fix_usb_permissions.sh
   ```

### **Phase 2: Mapping (Create Map)**

1. **Pi Terminal 1:**

   ```bash
   ./01_arduino_only.sh
   ```

2. **Pi Terminal 2:**

   ```bash
   ./02_lidar_only.sh
   # Choose option 1 (standard config)
   ```

3. **Laptop:**

   ```bash
   cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
   ./01_mapping_mode.sh
   ```

4. **Drive robot around to map area**

5. **Save map (on laptop):**
   ```bash
   ./03_save_map.sh
   ```

### **Phase 3: Navigation (Use Map)**

1. **Pi Terminal 1:**

   ```bash
   ./01_arduino_only.sh
   ```

2. **Pi Terminal 2:**

   ```bash
   ./02_lidar_only.sh
   ```

3. **Laptop:**

   ```bash
   ./02_navigation_mode.sh
   ```

4. **Set goals in RViz**

---

## Troubleshooting

### **If LiDAR fails:**

1. Try different baud rate: `./02_lidar_only.sh` → Option 2
2. Check power supply (voltage should be >1.1V)
3. Try different USB port
4. Run permission fix: `./fix_usb_permissions.sh`

### **If Arduino fails:**

1. Check which USB device: `ls -la /dev/ttyUSB*`
2. Arduino might be on /dev/ttyUSB1 instead of /dev/ttyUSB0

### **If laptop can't connect:**

1. Check network: both on same WiFi
2. Check ROS_DOMAIN_ID: should be same on both
3. Run diagnostics: `./04_diagnostics.sh`
