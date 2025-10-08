# Sensor Fusion and Drift Prevention - Complete Guide

## What We're Already Using

### ✅ robot_localization (Extended Kalman Filter)

You're **already using** a Kalman Filter! The `robot_localization` package implements an **Extended Kalman Filter (EKF)** that fuses multiple sensors.

**What it does:**
- Combines wheel odometry + IMU data optimally
- Statistically weights sensors based on their noise characteristics (covariances)
- Filters out sensor noise and drift
- Produces smooth, accurate odometry on `/odometry/filtered`

**Current status:** We just fixed the configuration to prevent drift!

---

## ROS2 Packages for Sensor Fusion

### 1. robot_localization (Currently Using!) ⭐

**Package:** `ros-humble-robot-localization`

**What it provides:**
- **EKF node** - Extended Kalman Filter
- **UKF node** - Unscented Kalman Filter (more accurate for non-linear systems)
- **navsat_transform** - Fuses GPS with local odometry

**Our current setup:**
```yaml
# ekf_config.yaml
ekf_filter_node:
  ros__parameters:
    # Fuses /odom (wheels) + /imu/data (orientation)
    # Outputs /odometry/filtered
```

**Pros:**
- ✅ Industry standard for ROS
- ✅ Handles multiple sensors
- ✅ Real-time performance
- ✅ Well-documented and maintained

**Cons:**
- ⚠️ Requires proper covariance tuning
- ⚠️ Can be sensitive to bad sensor data

---

### 2. SLAM Algorithms (Alternative/Complement)

#### A. slam_toolbox (Particle Filter + Graph Optimization)

**Package:** `ros-humble-slam-toolbox`

**What it does:**
- Builds a map while localizing the robot
- Uses **scan matching** to correct odometry drift
- Graph-based optimization (pose graph SLAM)

**How it helps with drift:**
- Compares laser scans to detect loop closures
- Corrects accumulated drift when revisiting known areas
- More robust than dead reckoning alone

**Setup:**
```bash
sudo apt install ros-humble-slam-toolbox

# Launch
ros2 launch slam_toolbox online_async_launch.py
```

**Use case:** When building maps AND navigating

---

#### B. cartographer (Multi-Sensor SLAM)

**Package:** `ros-humble-cartographer`

**What it does:**
- Google's SLAM solution
- Fuses LIDAR, IMU, and odometry
- Real-time 2D/3D mapping

**Pros:**
- ✅ Very accurate
- ✅ Handles large environments
- ✅ Built-in loop closure

**Cons:**
- ⚠️ More resource-intensive
- ⚠️ Complex configuration

---

### 3. AMCL (Monte Carlo Localization - Particle Filter)

**Package:** `ros-humble-nav2-amcl`

**What it does:**
- Localizes robot on a **pre-built map**
- Uses **particle filter** (not Kalman filter)
- Corrects odometry drift by matching laser scans to map

**How it helps:**
- Provides accurate pose estimate on known maps
- Eliminates long-term drift
- Used after SLAM when map is complete

**Setup:**
```bash
# After you have a map from SLAM:
ros2 launch nav2_bringup localization_launch.py map:=my_map.yaml
```

**Use case:** Navigation on a known map

---

## Numerical Methods for Drift Reduction

### 1. Dead Zone / Threshold (Already Applied! ✅)

**What we added:**
```python
# ros_arduino_bridge.py
encoder_threshold = 5  # Ignore encoder noise below 5 ticks
gyro_deadzone = 0.5    # Ignore gyro drift below 0.5°/s
```

**How it works:**
- Treats small sensor readings as zero
- Prevents noise from accumulating as motion
- **Most effective for stationary drift!**

---

### 2. Moving Average Filter (Simple Addition)

**Apply to encoder counts:**
```python
from collections import deque

# Add to __init__:
self.encoder_history = deque(maxlen=3)  # Keep last 3 readings

# In update_sensor_data, after getting encoder_counts:
self.encoder_history.append(self.encoder_counts.copy())
smoothed_counts = [
    sum(h[i] for h in self.encoder_history) / len(self.encoder_history)
    for i in range(4)
]
self.encoder_counts = [int(c) for c in smoothed_counts]
```

**Pros:** Simple, effective for noisy encoders  
**Cons:** Adds latency

---

### 3. Exponential Moving Average (Better Smoothing)

**For velocity estimates:**
```python
# Add to __init__:
self.alpha = 0.7  # Smoothing factor (0-1)
self.last_linear_vel = 0.0
self.last_angular_vel = 0.0

# In update_odometry, after calculating velocities:
linear_velocity = self.alpha * linear_velocity + (1 - self.alpha) * self.last_linear_vel
angular_velocity = self.alpha * angular_velocity + (1 - self.alpha) * self.last_angular_vel

self.last_linear_vel = linear_velocity
self.last_angular_vel = angular_velocity
```

**Pros:** Smooth output, low latency  
**Cons:** Can lag behind sudden changes

---

### 4. Median Filter (Outlier Rejection)

**For encoder readings:**
```python
import statistics

# Keep last 5 readings
self.encoder_window = {i: deque(maxlen=5) for i in range(4)}

# After reading encoders:
for i in range(4):
    self.encoder_window[i].append(counts[i])
    if len(self.encoder_window[i]) >= 3:
        # Use median to reject outliers
        self.encoder_counts[i] = int(statistics.median(self.encoder_window[i]))
```

**Pros:** Excellent for rejecting spikes  
**Cons:** More CPU intensive

---

### 5. Complementary Filter (IMU Fusion)

**Already built into MPU6050 DMP!**

The DMP (Digital Motion Processor) on your MPU6050 already runs a complementary filter:
- Fuses gyro (short-term accurate) + accelerometer (long-term stable)
- Outputs drift-free orientation
- This is why we use DMP orientation in the EKF!

---

## Recommended Setup for Your Robot

### Current Configuration (After Our Fixes)

```
┌─────────────┐
│   Arduino   │
└──────┬──────┘
       │
       ├─→ Encoders ──→ /odom (position + velocity)
       │              ↓ encoder_threshold = 5
       │
       └─→ MPU6050 ──→ /imu/data (DMP orientation only)
                      ↓ gyro_deadzone = 0.5°/s
                      ↓
                      ↓
           ┌──────────▼──────────┐
           │  robot_localization │
           │   (EKF Filter)      │
           └──────────┬──────────┘
                      │
                      ├─→ /odometry/filtered (corrected pose)
                      └─→ TF: odom → base_link
                      
                      ↓
                      
           ┌──────────▼──────────┐
           │    SLAM Toolbox     │ (Optional)
           │  (Scan Matching)    │
           └──────────┬──────────┘
                      │
                      └─→ /map
                      └─→ TF: map → odom
```

---

## Testing the Fixes

### Test 1: Stationary Drift Test (Critical!)

```bash
# Terminal 1: Launch with fixes
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Terminal 2: Record starting position
ros2 topic echo /odometry/filtered --once | grep -A3 "position:"

# Wait 5 minutes (robot stationary!)

# Terminal 3: Check final position
ros2 topic echo /odometry/filtered --once | grep -A3 "position:"

# SUCCESS: Position should be within 1-2cm of start
# FAILURE: Position drifts > 5cm → more tuning needed
```

### Test 2: In-Place Rotation Test

```bash
# Slowly rotate robot 360° in place
# Monitor /odometry/filtered

# Expected: position.x and position.y stay constant
# If position "walks" during rotation → IMU position offset wrong
```

### Test 3: Straight Line Test

```bash
# Drive straight for 2 meters
# View in RViz with Fixed Frame = map

# Expected: Path is straight
# If path curves → systematic error in wheel diameter or base_width
```

---

## Advanced: Add SLAM for Drift Elimination

### Why SLAM Helps

Even with perfect sensor fusion, **dead reckoning always drifts** over long distances. SLAM corrects this by:

1. **Scan matching** - Compares current laser scan to previous scans
2. **Loop closure** - Detects when you return to a known location
3. **Graph optimization** - Corrects entire trajectory retroactively

### Quick Setup

```bash
# Terminal 1: Hardware + EKF
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
ros2 launch ros_arduino_bridge sensor_fusion.launch.py

# Terminal 2: LIDAR
ros2 launch sllidar_ros2 sllidar_launch.py

# Terminal 3: SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 4: RViz
rviz2
# Set Fixed Frame: map
# Add: Map, LaserScan, TF, Odometry
```

**Result:** Map stays perfectly still, robot localizes accurately!

---

## Comparison: Different Approaches

| Method | Drift Prevention | Accuracy | CPU Usage | Setup Complexity |
|--------|-----------------|----------|-----------|------------------|
| **EKF (robot_localization)** ⭐ | Good (short term) | High | Low | Medium |
| Dead Zone / Threshold | Excellent (stationary) | Medium | Minimal | Easy |
| Moving Average | Good | Medium | Low | Easy |
| Median Filter | Excellent (spikes) | High | Medium | Easy |
| **SLAM (slam_toolbox)** | Excellent (long term) | Very High | High | Medium |
| AMCL + Pre-built Map | Excellent | Very High | Medium | Hard |

---

## What's Already Preventing Drift

### ✅ Applied in This Session

1. **Encoder threshold** (5 ticks) - Prevents encoder noise integration
2. **Gyro dead zone** (0.5°/s) - Eliminates gyro drift when stationary
3. **Zero velocity publishing** - Anchors EKF when robot is stopped
4. **DMP orientation only** - Uses most stable IMU output, ignores noisy gyro/accel
5. **Proper covariances** - Tells EKF to trust encoders more than raw gyro
6. **IMU calibration** - Removes systematic gyro bias at startup

---

## Next Steps to Further Reduce Drift

### Option 1: Add Moving Average (Easy)

Add encoder smoothing to reduce noise:

```python
# In ros_arduino_bridge.py, add after line ~790

from collections import deque

# In __init__ (after line 115):
self.encoder_history = deque(maxlen=3)

# In update_sensor_data, after line 825:
if len(counts) == 4:
    self.encoder_history.append(counts.copy())
    if len(self.encoder_history) >= 2:
        # Average last readings
        smoothed = [
            sum(h[i] for h in self.encoder_history) / len(self.encoder_history)
            for i in range(4)
        ]
        self.encoder_counts = [int(c) for c in smoothed]
    else:
        self.encoder_counts = counts
```

### Option 2: Add SLAM (Recommended)

Eliminate long-term drift completely:

```bash
# Always run SLAM during navigation
ros2 launch slam_toolbox online_async_launch.py
```

### Option 3: Calibrate Robot Parameters

Fine-tune `robot_params.yaml`:

```bash
# Measure actual wheel diameter
# Measure actual track width
# Update config and test again
```

---

## Summary

### What RViz Does
- ❌ Does NOT filter or process sensor data
- ✅ Only visualizes data from topics

### What Actually Filters Data
- ✅ **robot_localization (EKF)** - Your main filter (already using!)
- ✅ **Dead zones** - Threshold-based noise rejection (added!)
- ✅ **Moving averages** - Smoothing filters (optional)
- ✅ **SLAM** - Map-based drift correction (recommended)

### Your Current Stack

```
Raw Sensors → Dead Zones → EKF → Filtered Odometry → SLAM → Map
(noisy)       (reduce)     (fuse)  (smooth)           (correct)  (accurate)
```

---

## Quick Decision Guide

**Problem:** Robot drifts when stationary  
**Solution:** ✅ Already applied (dead zones + zero velocity)

**Problem:** Robot drifts during short drives (<5 minutes)  
**Solution:** Add moving average filter + tune EKF covariances

**Problem:** Robot drifts over long distances (>10 minutes)  
**Solution:** Add SLAM (slam_toolbox)

**Problem:** Need perfect localization on known map  
**Solution:** Use AMCL with pre-built map

---

## Testing Your Fixes

Run this test script on the Pi:

```bash
#!/bin/bash
# test_drift.sh

echo "=== Drift Test Suite ==="
echo ""
echo "Test 1: Stationary Drift (5 minutes)"
echo "Starting position:"
ros2 topic echo /odometry/filtered --once | grep -A3 "position:"
echo ""
echo "Waiting 5 minutes... (keep robot still!)"
sleep 300
echo ""
echo "Ending position:"
ros2 topic echo /odometry/filtered --once | grep -A3 "position:"
echo ""
echo "If drift < 2cm in x,y: PASS ✓"
echo "If drift > 5cm: FAIL ✗ (needs more tuning)"
```

**Expected result after our fixes:** Drift < 2cm over 5 minutes when stationary!

---

Need help implementing any of these? Let me know which approach you want to try!
