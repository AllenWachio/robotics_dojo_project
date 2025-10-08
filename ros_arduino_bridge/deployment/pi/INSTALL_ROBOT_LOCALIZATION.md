# Installing robot_localization on Raspberry Pi

## Error Message

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
"package 'robot_localization' not found"
```

## Solution

The `robot_localization` package is required for EKF sensor fusion but is not installed by default.

### Install Command

On your Raspberry Pi, run:

```bash
sudo apt update
sudo apt install ros-humble-robot-localization
```

This will install the package from the ROS 2 Humble apt repository.

---

## Verification

After installation, verify it's installed:

```bash
ros2 pkg list | grep robot_localization
```

**Expected output:**

```
robot_localization
```

---

## Now Test EKF Mode

```bash
cd ~/ros2_ws
./src/robotics_dojo_project/ros_arduino_bridge/deployment/scripts/pi/01_arduino_only.sh --ekf
```

Should now work without the package error!

---

## Alternative: Use Without EKF

If you don't want to install `robot_localization` right now, just run without the `--ekf` flag:

```bash
./src/robotics_dojo_project/ros_arduino_bridge/deployment/scripts/pi/01_arduino_only.sh
```

The updated script now checks if the package is installed and automatically falls back to standard mode if not.

---

## Package Size

The `robot_localization` package is approximately 1-2 MB and is lightweight.

---

## What It Provides

Once installed, you get:

- `ekf_node` - Extended Kalman Filter for sensor fusion
- `ukf_node` - Unscented Kalman Filter (more advanced)
- `navsat_transform_node` - For GPS integration

For your robot, we only use the `ekf_node`.

---

**Next Step:** Install the package on the Raspberry Pi, then test again! 🚀
