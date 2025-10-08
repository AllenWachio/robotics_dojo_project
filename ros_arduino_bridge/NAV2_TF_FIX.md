# Nav2 Abortion Issue - TF Transform Problem

**Date:** October 8, 2025  
**Branch:** merge_All  
**Issue:** Nav2 aborts immediately when trying to start navigation

---

## 🔴 Problem Identified

Nav2 is aborting because of **TF transform chain issues**. Specifically:

### **Current Setup Issues:**

1. **Odometry Topic Mismatch:**
   - Nav2 config expects: `/odom`
   - EKF publishes: `/odometry/filtered`
   - AMCL needs filtered odometry for accurate localization!

2. **TF Chain Requirements:**
   ```
   Nav2 needs complete chain:
   map → odom → base_link → sensors
   
   Current state:
   - EKF publishes: odom → base_link ✅
   - AMCL should publish: map → odom ❌ (but needs correct odom topic!)
   ```

3. **Transform Timeout:**
   - Nav2 waits for transforms
   - If AMCL doesn't get correct odometry, it can't publish `map → odom`
   - Result: Nav2 aborts due to missing transforms

---

## 🔍 Root Cause Analysis

### **EKF Configuration (CORRECT):**
```yaml
# ros_arduino_bridge/config/ekf_config.yaml
odom0: /odom                    # INPUT: Raw encoder odometry
world_frame: odom               # OUTPUT TF: odom → base_link
# Publishes /odometry/filtered  # OUTPUT TOPIC: Fused odometry
```

### **Nav2 Configuration (WRONG - needs update):**
```yaml
# deployment/laptop/config/nav2_params.yaml
odom_topic: /odom               # ❌ WRONG! Should be /odometry/filtered
```

### **Why This Breaks Nav2:**

1. **AMCL subscribes to odometry topic** specified in nav2_params.yaml
2. **AMCL receives raw `/odom`** (not filtered!) → inaccurate localization
3. **AMCL's map→odom transform** is based on bad data → drift during turns
4. **Nav2 sees bad localization** → aborts mission

---

## ✅ Solution

### **Fix 1: Update Nav2 to use filtered odometry**

**File:** `deployment/laptop/config/nav2_params.yaml`

**Change:**
```yaml
# OLD (line 6):
odom_topic: /odom

# NEW:
odom_topic: /odometry/filtered
```

**Also check line 309:**
```yaml
# OLD:
odom_topic: "odom"

# NEW:
odom_topic: "/odometry/filtered"
```

### **Fix 2: Verify AMCL Configuration**

AMCL needs to know about the filtered odometry. In `nav2_params.yaml`, ensure:

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"
    scan_topic: "/scan"
    
    # CRITICAL: This ensures AMCL uses the right odometry
    # The odom_topic at the top level applies here
```

### **Fix 3: Check TF Tree**

After fixes, verify TF chain:

```bash
# On laptop (while robot and nav2 running):
ros2 run tf2_tools view_frames

# Should show:
# map (from amcl)
#   → odom (from ekf)
#       → base_link
#           → laser, imu_link, etc.
```

---

## 🛠️ Implementation Steps

### **Step 1: Update nav2_params.yaml**

```bash
# Edit the file
cd ~/robotics_dojo_project/ros_arduino_bridge/deployment/laptop/config
nano nav2_params.yaml

# Find line 6 and change:
odom_topic: /odom
# To:
odom_topic: /odometry/filtered

# Find line 309 and change:
odom_topic: "odom"
# To:
odom_topic: "/odometry/filtered"
```

### **Step 2: Verify EKF is Running**

```bash
# Check EKF node
ros2 node list | grep ekf

# Check filtered odometry topic
ros2 topic echo /odometry/filtered

# Check TF from EKF
ros2 run tf2_ros tf2_echo odom base_link
```

### **Step 3: Test Navigation**

```bash
# Launch navigation
ros2 launch ros_arduino_bridge laptop_navigation.launch.py

# Check AMCL is getting odometry
ros2 topic info /odometry/filtered

# Check TF chain
ros2 run tf2_tools view_frames

# Set initial pose in RViz (2D Pose Estimate button)
# Then try setting a goal (2D Nav Goal button)
```

---

## 📊 Expected TF Tree After Fix

```
map (from amcl)
  ↓
odom (from ekf_filter_node)
  ↓
base_link (from ekf_filter_node)
  ↓
├─ laser (from robot_state_publisher)
├─ imu_link (from robot_state_publisher)
└─ wheels (from robot_state_publisher)
```

---

## 🚨 Common Nav2 Abortion Reasons

### **1. Missing map → odom transform**
- **Symptom:** "Timed out waiting for transform from base_link to map"
- **Cause:** AMCL not running or not localized
- **Fix:** Set initial pose in RViz using "2D Pose Estimate"

### **2. Incorrect odometry topic**
- **Symptom:** AMCL not publishing map→odom, or publishing bad transform
- **Cause:** AMCL subscribed to wrong odom topic (raw instead of filtered)
- **Fix:** Update nav2_params.yaml to use /odometry/filtered

### **3. TF timeout**
- **Symptom:** "Extrapolation into the future requested"
- **Cause:** Time synchronization issues or missing transforms
- **Fix:** 
  - Check time sync between Pi and laptop
  - Verify all TF publishers are running
  - Check TF buffer size in nav2_params.yaml

### **4. No map loaded**
- **Symptom:** "Map server not available"
- **Cause:** Map file doesn't exist or wrong path
- **Fix:** Verify map file in ~/ros2_ws/maps/

### **5. Costmap initialization failure**
- **Symptom:** "Failed to get map from map server"
- **Cause:** Map server not running or publishing
- **Fix:** 
  - ros2 topic echo /map
  - Check map_server node is running

---

## 🔧 Diagnostic Commands

### **Check TF Chain:**
```bash
# View all transforms
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link

# Check if AMCL is publishing
ros2 topic list | grep amcl
ros2 topic echo /amcl_pose
```

### **Check Odometry:**
```bash
# Raw odometry (from encoders)
ros2 topic echo /odom

# Filtered odometry (from EKF)
ros2 topic echo /odometry/filtered

# Compare them - filtered should have less drift
```

### **Check Nav2 Status:**
```bash
# Check all nav2 nodes
ros2 node list | grep nav2

# Check nav2 lifecycle state
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /bt_navigator

# Check nav2 diagnostics
ros2 topic echo /diagnostics
```

### **Check AMCL:**
```bash
# Check AMCL node
ros2 node list | grep amcl

# Check particle cloud (should see particles in RViz)
ros2 topic echo /particlecloud

# Check AMCL pose
ros2 topic echo /amcl_pose
```

---

## 📝 Quick Fix Checklist

Before running Nav2, verify:

- [ ] EKF is running and publishing /odometry/filtered
- [ ] TF shows: odom → base_link (from EKF)
- [ ] Map server is loaded with your saved map
- [ ] nav2_params.yaml uses `/odometry/filtered` for odom_topic
- [ ] AMCL is configured correctly
- [ ] Initial pose set in RViz (2D Pose Estimate)
- [ ] Robot localized on map (particles converged in RViz)
- [ ] TF shows complete chain: map → odom → base_link

---

## 🎯 Testing After Fix

### **Test 1: Verify TF Chain**
```bash
ros2 run tf2_tools view_frames
# Check PDF output for complete chain
```

### **Test 2: Set Initial Pose**
```bash
# In RViz:
# 1. Click "2D Pose Estimate"
# 2. Click on map where robot is
# 3. Drag to set orientation
# 4. Watch particle cloud converge
```

### **Test 3: Send Nav Goal**
```bash
# In RViz:
# 1. Click "2D Nav Goal"
# 2. Click destination on map
# 3. Drag to set final orientation
# 4. Watch robot plan path and execute
```

### **Test 4: Monitor During Navigation**
```bash
# Watch filtered odometry (should be smooth)
ros2 topic echo /odometry/filtered

# Watch TF from map to base_link
ros2 run tf2_ros tf2_echo map base_link

# Watch nav2 feedback
ros2 topic echo /navigate_to_pose/_action/feedback
```

---

## 🏆 Success Criteria

After applying fixes, you should see:

1. **AMCL converges** - particle cloud tightens around robot
2. **Nav2 accepts goals** - no immediate abortion
3. **Path planning works** - see planned path in RViz
4. **Robot follows path** - smooth motion to goal
5. **TF chain complete** - no transform warnings
6. **No drift** - robot stays localized on map during turns

---

## 📚 Related Files to Check

1. `ros_arduino_bridge/config/ekf_config.yaml` - EKF configuration ✅ CORRECT
2. `deployment/laptop/config/nav2_params.yaml` - Nav2 configuration ❌ NEEDS FIX
3. `deployment/laptop/launch/laptop_navigation.launch.py` - Nav2 launch file
4. `ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py` - Odometry publisher
5. `ros_arduino_bridge/launch/sensor_fusion.launch.py` - EKF launch

---

**Next Step:** Apply Fix 1 to nav2_params.yaml and test!
