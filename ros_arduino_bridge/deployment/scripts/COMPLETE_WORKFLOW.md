# Complete Robot Workflow Guide

## 🎉 Major Update: Dual Localization Support!

Your robot now supports **TWO localization methods** for autonomous navigation:

### **Map Format Overview**

When you save a map, you now get **FOUR files**:

1. **Old Format (for AMCL):**

   - `map_name.pgm` - Image of the map
   - `map_name.yaml` - Metadata file

2. **Serialized Format (for SLAM Toolbox):**
   - `map_name.data` - Pose graph data
   - `map_name.posegraph` - Map structure

### **Which Localization Method to Use?**

| Method           | Pros                                               | Cons                                 | Best For                             |
| ---------------- | -------------------------------------------------- | ------------------------------------ | ------------------------------------ |
| **AMCL**         | Industry standard, well-tested, works with any map | Can drift over time, particle filter | Quick navigation, familiar workflows |
| **SLAM Toolbox** | More accurate, can refine map, better loop closure | Newer technology, more CPU intensive | Long-term operation, map refinement  |

**💡 Recommendation:** Try SLAM Toolbox localization first (Option C)! It provides the best results.

---

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

### **Laptop Scripts:**

- ✅ `01_mapping_mode.sh` - Create new maps with SLAM
- ✅ `02_navigation_mode.sh` - Navigate with AMCL localization
- ✅ `02b_slam_localization_mode.sh` - 🆕 **NEW:** SLAM Toolbox localization only
- ✅ `02c_slam_navigation_mode.sh` - 🆕 **NEW:** Full navigation with SLAM localization
- ✅ `03_save_map.sh` - 🔄 **UPDATED:** Now saves BOTH map formats!
- ✅ `04_diagnostics.sh` - System diagnostics

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
   # Enter a unique name for your map when prompted
   # Script will check if name already exists and ask for different name if needed
   #
   # 🎯 NEW: Script now saves BOTH map formats:
   #    - Old format (.pgm + .yaml) - for AMCL/Nav2
   #    - Serialized format (.data + .posegraph) - for SLAM Toolbox localization
   # This gives you maximum flexibility for navigation!
   ```

### **Phase 3: Navigation (Use Map)**

🆕 **You now have THREE navigation options!**

#### **Option A: AMCL Navigation (Original Method)**

Uses AMCL for localization with old format maps (.pgm + .yaml)

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
   ./02_navigation_mode.sh [map_name]
   # OR just ./02_navigation_mode.sh and choose from available maps
   ```

4. **Set initial pose in RViz (2D Pose Estimate)**
5. **Set goals in RViz (2D Goal Pose or Nav2 Goal)**

---

#### **Option B: SLAM Toolbox Localization Only (No Navigation)**

Uses SLAM Toolbox for localization with serialized maps (.data + .posegraph)
Good for testing localization before running full navigation

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
   ./02b_slam_localization_mode.sh [map_name]
   # Uses serialized maps for SLAM Toolbox localization
   ```

4. **Set initial pose in RViz (2D Pose Estimate)**
5. **Robot localizes on saved map - you can see its position update**

---

#### **Option C: Full SLAM Navigation (Recommended!)**

Uses SLAM Toolbox for localization + Nav2 for autonomous navigation
Best of both worlds!

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
   ./02c_slam_navigation_mode.sh [map_name]
   # Full navigation with SLAM Toolbox localization
   ```

4. **Set initial pose in RViz (2D Pose Estimate)**
5. **Wait for localization to stabilize**
6. **Set goals in RViz (2D Goal Pose or Nav2 Goal) - Robot navigates autonomously!**

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
