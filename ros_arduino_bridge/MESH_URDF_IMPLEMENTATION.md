# Mesh-Based URDF Implementation Plan

## Branch: mesh_based_urdf

Created from: `making_pytrees_work_well`
Purpose: Complete mesh-based URDF with all sensors including IMU

---

## Current Status

### ✅ Already Implemented (Mesh-Based)
The URDF already uses STL meshes for:

1. **base_link** - Main robot chassis
   - File: `meshes/base_link.stl`
   - Scale: 0.001 (mm → m)

2. **fore_right_wheel_1** - Front right wheel
   - File: `meshes/fore_right_wheel_1.stl`
   - Joint: continuous (rotates freely)

3. **fore_left_wheel_1** - Front left wheel
   - File: `meshes/fore_left_wheel_1.stl`
   - Joint: continuous

4. **back_right_wheel_1** - Back right wheel
   - File: `meshes/back_right_wheel_1.stl`
   - Joint: continuous

5. **back_left_wheel_1** - Back left wheel
   - File: `meshes/back_left_wheel_1.stl`
   - Joint: continuous

6. **lidar_1** - RPLidar sensor
   - File: `meshes/lidar_1.stl`
   - Joint: fixed
   - Child: `laser` frame for SLAM

### 📦 Available Mesh Files
```
meshes/
├── base_link.stl           (3 KB)
├── fore_left_wheel_1.stl   (20 KB)
├── fore_right_wheel_1.stl  (20 KB)
├── back_left_wheel_1.stl   (20 KB)
├── back_right_wheel_1.stl  (20 KB)
├── lidar_1.stl             (20 KB)
└── imu_1.stl               (exists, not yet in URDF!)
```

---

## What Needs to Be Added

### ⚠️ Missing: IMU Link
The IMU mesh exists (`imu_1.stl`) but is **not defined in the URDF**.

**Requirements:**
- Add `imu_link` with visual and collision meshes
- Position: 3cm below `base_link` center (matching TF transform)
- Joint type: fixed (doesn't move relative to base)
- Mesh: `meshes/imu_1.stl`

**Expected Result:**
```xml
<link name="imu_link">
  <inertial>
    <!-- IMU is very light, ~5 grams -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.005"/>
    <inertia ixx="0.000001" iyy="0.000001" izz="0.000001" 
             ixy="0.0" iyz="0.0" ixz="0.0"/>
  </inertial>
  <visual>
    <origin xyz="..." rpy="0 0 0"/>
    <geometry>
      <mesh filename="file://$(find ros_arduino_bridge)/meshes/imu_1.stl" 
            scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="silver"/>
  </visual>
  <collision>
    <origin xyz="..." rpy="0 0 0"/>
    <geometry>
      <mesh filename="file://$(find ros_arduino_bridge)/meshes/imu_1.stl" 
            scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
</link>

<joint name="imu_joint" type="fixed">
  <origin xyz="0 0 -0.03" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="imu_link"/>
</joint>
```

---

## Optional Enhancements

### Camera Link (Future)
If camera mesh exists or will be created:
```xml
<link name="camera_link">
  <!-- Camera mounted on top/front of robot -->
</link>

<joint name="camera_joint" type="fixed">
  <origin xyz="0.15 0 0.20" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="camera_link"/>
</joint>
```

---

## TF Tree After IMU Addition

```
base_link (root)
 ├─ fore_right_wheel_1 (continuous joint)
 ├─ fore_left_wheel_1 (continuous joint)
 ├─ back_right_wheel_1 (continuous joint)
 ├─ back_left_wheel_1 (continuous joint)
 ├─ lidar_1 (fixed joint)
 │   └─ laser (fixed joint, for SLAM)
 └─ imu_link (fixed joint) ← TO BE ADDED
```

**Complete TF Tree (with odom/map):**
```
map (SLAM)
 └─ odom (EKF)
     └─ base_link (EKF publishes)
         ├─ imu_link ← NEW! (defined in URDF, not static_transform_publisher)
         ├─ lidar_1
         │   └─ laser
         ├─ fore_right_wheel_1
         ├─ fore_left_wheel_1
         ├─ back_right_wheel_1
         └─ back_left_wheel_1
```

---

## Benefits of Mesh-Based URDF

### Visual Accuracy
- ✅ Exact representation of real robot geometry
- ✅ Proper proportions in RViz/Gazebo
- ✅ Easier debugging of TF issues (see actual robot shape)

### Collision Detection
- ✅ Accurate collision geometry
- ✅ Better path planning in Nav2
- ✅ Simulation accuracy

### Professional Appearance
- ✅ Realistic visualization
- ✅ Better for demonstrations
- ✅ Easier for team understanding

---

## Implementation Steps

### Step 1: Add IMU Link to URDF ✅ (Ready to implement)
- [ ] Calculate correct origin offset for visual/collision
- [ ] Add inertial properties (mass, inertia)
- [ ] Add visual geometry with mesh
- [ ] Add collision geometry with mesh
- [ ] Add fixed joint from base_link

### Step 2: Remove Static Transform Publisher (Optional)
Since `imu_link` will be defined in URDF, we can optionally remove the static_transform_publisher from launch files:
- Current: `static_transform_publisher` in `slam_with_sensor_fusion.launch.py`
- After URDF addition: Not needed (robot_state_publisher handles it)

**Note:** Keeping both won't cause conflicts (same transform), but cleaner to use URDF only.

### Step 3: Test in RViz
```bash
ros2 launch ros_arduino_bridge arduino_bridge.py
```
Verify:
- [ ] All links visible
- [ ] IMU link appears at correct position
- [ ] TF tree shows all links
- [ ] No TF errors/warnings

### Step 4: Test with Sensor Fusion
```bash
ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
```
Verify:
- [ ] IMU data published to `/imu/data`
- [ ] EKF receives IMU data
- [ ] TF tree complete: map → odom → base_link → imu_link
- [ ] No TF conflicts

---

## Files to Modify

### 1. `urdf/new_robot_urdf.xacro`
**Action:** Add IMU link and joint
**Location:** Before closing `</robot>` tag

### 2. `launch/slam_with_sensor_fusion.launch.py` (Optional)
**Action:** Remove static_transform_publisher for imu_link
**Reason:** Now defined in URDF, robot_state_publisher handles it

### 3. `launch/sensor_fusion.launch.py` (Optional)
**Action:** Remove static_transform_publisher for imu_link
**Reason:** Same as above

---

## Testing Checklist

### Visual Inspection
- [ ] `ros2 run tf2_tools view_frames` - Generate TF tree PDF
- [ ] Open in RViz, enable RobotModel
- [ ] Verify IMU visible at correct position (3cm below base)
- [ ] Check all meshes load correctly (no missing mesh errors)

### Functional Testing
- [ ] Arduino bridge publishes joint states
- [ ] robot_state_publisher publishes all TF transforms
- [ ] IMU data publishes to `/imu/data`
- [ ] EKF fuses encoder + IMU data
- [ ] SLAM uses filtered odometry
- [ ] No TF warnings/errors in logs

### Integration Testing
- [ ] Teleop mode works (standalone, no EKF)
- [ ] SLAM mode works (with EKF sensor fusion)
- [ ] Navigation works (Nav2 with AMCL)
- [ ] Map doesn't break during turns

---

## Documentation to Update

After implementation:
- [ ] Update `README.md` with mesh-based URDF info
- [ ] Update `IMU_SENSOR_FUSION_STATUS.md` (remove static_transform note)
- [ ] Create `MESH_URDF_GUIDE.md` (this document)
- [ ] Update launch file documentation

---

## Next Steps

1. **Immediate:** Add IMU link to URDF
2. **Test:** Verify in RViz and with hardware
3. **Clean up:** Remove redundant static transforms (optional)
4. **Document:** Update all relevant docs
5. **Push:** Commit to `mesh_based_urdf` branch

---

**Created:** January 2025
**Branch:** mesh_based_urdf
**Status:** ⏳ Planning complete, ready to implement
