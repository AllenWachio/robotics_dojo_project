# Merge Strategy: z_axix (Sensor Fusion) + first-pytrees (Behavior Tree)

## Problem
You have 190 merge conflicts because both branches have divergent histories.

## Solution Options

### **Option A: Keep Current Branch, Cherry-Pick Sensor Fusion Files (RECOMMENDED)**

This keeps your behavior tree work and adds only the sensor fusion changes:

```bash
# Abort current merge
git merge --abort

# Create backup branch
git branch backup-first-pytrees

# Cherry-pick specific sensor fusion commits from z_axix
git cherry-pick 73eeda9  # Fix double integration drift
git cherry-pick f0f2923  # IMU implementation and motor fixes
git cherry-pick d79da32  # Fix dual TF publishing
```

If cherry-pick has conflicts, manually copy only these critical files from z_axix:

```bash
# Sensor fusion files to keep from z_axix
git checkout z_axix -- ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py
git checkout z_axix -- ros_arduino_bridge/config/ekf_config.yaml
git checkout z_axix -- ros_arduino_bridge/launch/sensor_fusion.launch.py
git checkout z_axix -- ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py
```

### **Option B: Force Merge with "Ours" Strategy**

Keep first-pytrees version for most files, only take z_axix for sensor fusion:

```bash
# Accept current branch (ours) for all conflicts
git checkout --ours .
git add .

# Now manually take z_axix version for sensor fusion files
git checkout --theirs ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py
git checkout --theirs ros_arduino_bridge/config/ekf_config.yaml
git checkout --theirs ros_arduino_bridge/launch/arduino_bridge.py
git checkout --theirs ros_arduino_bridge/launch/sensor_fusion.launch.py
git checkout --theirs ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py
git checkout --theirs ros_arduino_bridge/launch/test_teleop.launch.py
git checkout --theirs ros_arduino_bridge/launch/full_slam_test.launch.py

# Add the sensor fusion files
git add ros_arduino_bridge/

# Complete the merge
git commit -m "Merge z_axix sensor fusion with first-pytrees behavior tree"
```

### **Option C: Start Fresh - Create Unified Branch**

```bash
# Abort current merge
git merge --abort

# Create new branch from first-pytrees
git checkout -b unified-sensor-fusion-and-behavior

# Manually copy sensor fusion files from z_axix
git show z_axix:ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py > ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py
git show z_axix:ros_arduino_bridge/config/ekf_config.yaml > ros_arduino_bridge/config/ekf_config.yaml
git show z_axix:ros_arduino_bridge/launch/arduino_bridge.py > ros_arduino_bridge/launch/arduino_bridge.py
git show z_axix:ros_arduino_bridge/launch/sensor_fusion.launch.py > ros_arduino_bridge/launch/sensor_fusion.launch.py
git show z_axix:ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py > ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py

# Commit the unified changes
git add ros_arduino_bridge/
git commit -m "Add sensor fusion from z_axix to unified branch"
```

## Critical Files from z_axix (Sensor Fusion)

These files MUST come from z_axix branch:

1. **`ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py`**
   - IMU degree-to-radian conversion fix
   - M4 motor debugging
   - Velocity-only odometry
   - IMU calibration

2. **`ros_arduino_bridge/config/ekf_config.yaml`**
   - Encoder velocities only (no position)
   - IMU yaw only (no angular velocity)
   - Optimal covariances

3. **`ros_arduino_bridge/launch/*.py`**
   - `publish_tf: False` in arduino_bridge when using EKF
   - Proper sensor fusion launch configuration

## Files to Keep from first-pytrees (Behavior Tree)

1. All behavior_tree/ directory files
2. Py_trees integration
3. Mission planning code

## Recommended Command Sequence

```bash
# Execute Option B (Force merge with selective files)
cd "e:\Comp_Stuff\Glenn_s Shtuff\Projects\RDJ_2025\Allen_Repo\robotics_dojo_project"

# Accept ours (first-pytrees) for everything
git checkout --ours .
git add .

# Take theirs (z_axix) for sensor fusion
git checkout --theirs ros_arduino_bridge/ros_arduino_bridge/ros_arduino_bridge.py
git checkout --theirs ros_arduino_bridge/config/ekf_config.yaml
git checkout --theirs ros_arduino_bridge/launch/arduino_bridge.py
git checkout --theirs ros_arduino_bridge/launch/sensor_fusion.launch.py
git checkout --theirs ros_arduino_bridge/launch/slam_with_sensor_fusion.launch.py
git checkout --theirs ros_arduino_bridge/launch/test_teleop.launch.py
git checkout --theirs ros_arduino_bridge/launch/full_slam_test.launch.py

git add ros_arduino_bridge/
git commit -m "Merge z_axix sensor fusion fixes with first-pytrees behavior tree"

# Push the merged result
git push origin first-pytrees
```
