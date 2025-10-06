# Testing Checklist - RPI Camera Package

Use this checklist to verify the implementation on your actual hardware.

---

## ✅ Pre-Flight Checks

### Raspberry Pi Setup

- [ ] ROS2 Humble installed
- [ ] Camera Module 2 physically connected to CSI port
- [ ] Legacy camera enabled in raspi-config
- [ ] I2C and SPI enabled in raspi-config
- [ ] User added to video group: `groups | grep video`
- [ ] Camera device exists: `ls -l /dev/video0`
- [ ] v4l2_camera installed: `ros2 pkg list | grep v4l2`
- [ ] image_transport_plugins installed
- [ ] Network configured (ROS_DOMAIN_ID=42, etc.)

### Laptop Setup

- [ ] ROS2 Humble installed
- [ ] OpenCV installed: `python3 -c "import cv2; print(cv2.__version__)"`
- [ ] PyTorch installed: `python3 -c "import torch; print(torch.__version__)"`
- [ ] PIL installed: `python3 -c "import PIL; print(PIL.__version__)"`
- [ ] cv_bridge installed: `ros2 pkg list | grep cv_bridge`
- [ ] image_transport_plugins installed
- [ ] Network configured (same as Pi)

### Network

- [ ] Both on same network
- [ ] Can ping Pi from laptop: `ping <pi_ip>`
- [ ] Can ping laptop from Pi: `ping <laptop_ip>`
- [ ] ROS_DOMAIN_ID=42 on both
- [ ] ROS_LOCALHOST_ONLY=0 on both

---

## 🔨 Build Test

### On Both Machines

```bash
cd ~/ros2_ws
colcon build --packages-select rpi_camera_package --symlink-install
source install/setup.bash
```

- [ ] Build succeeds without errors
- [ ] Executables exist: `ros2 pkg executables rpi_camera_package`
- [ ] Shows: `color_detection_node`, `disease_detection_node`
- [ ] Model file installed: `ls ~/ros2_ws/install/rpi_camera_package/share/rpi_camera_package/models/model_ft_gpu.pth`

---

## 🧪 Hardware Tests

### Test 1: Camera Detection (Pi Only)

```bash
# Check device
ls -l /dev/video0

# List capabilities
v4l2-ctl --list-devices

# Check formats
v4l2-ctl -d /dev/video0 --list-formats-ext
```

**Expected**:

- [x] `/dev/video0` exists
- [x] Shows "mmal" or "bcm2835" device
- [x] Supports YUYV or MJPEG formats

**Result**: \***\*\_\_\_\*\***

---

### Test 2: Basic Camera Launch (Pi Only)

```bash
# Terminal 1 on Pi
ros2 launch rpi_camera_package camera_publisher.launch.py
```

**Expected Output**:

```
[camera-1] [INFO] [camera]: Camera node started
[camera-1] [INFO] [camera]: Publishing to /camera/image_raw
```

**Checks**:

- [ ] No errors in terminal
- [ ] Node starts successfully
- [ ] Check topics: `ros2 topic list` shows `/camera/image_raw`
- [ ] Check rate: `ros2 topic hz /camera/image_raw` shows ~20-30 Hz

**Result**: \***\*\_\_\_\*\***

---

### Test 3: Compressed Camera Launch (Pi Only)

```bash
# Terminal 1 on Pi
ros2 launch rpi_camera_package camera_compressed.launch.py
```

**Expected**:

- [ ] Camera node starts
- [ ] Republisher node starts
- [ ] `/camera/image_raw/compressed` topic exists
- [ ] Rate: `ros2 topic hz /camera/image_raw/compressed` shows ~20-30 Hz

**Result**: \***\*\_\_\_\*\***

---

### Test 4: Network Discovery

```bash
# Terminal on Pi (with camera running)
ros2 node list

# Terminal on Laptop (camera should be running on Pi)
ros2 node list
```

**Expected**:

- [ ] Pi sees: `/camera/camera`
- [ ] Laptop also sees: `/camera/camera` (from Pi)
- [ ] Both machines see same nodes

**Result**: \***\*\_\_\_\*\***

---

### Test 5: Image Reception (Laptop)

```bash
# Pi: Keep camera running
# Terminal 1 on Laptop
ros2 topic echo /camera/image_raw/compressed --once
```

**Expected**:

- [ ] Receives compressed image data
- [ ] Shows header with timestamp
- [ ] Shows data array with image bytes

**Result**: \***\*\_\_\_\*\***

---

### Test 6: Color Detection (Full System)

```bash
# Pi Terminal 1
ros2 launch rpi_camera_package camera_compressed.launch.py

# Laptop Terminal 1
ros2 launch rpi_camera_package color_detection.launch.py
```

**Expected**:

- [ ] Laptop node starts with "✅ Subscribed to COMPRESSED images"
- [ ] OpenCV window opens showing camera feed
- [ ] Stats printed every 5 seconds: "📊 Stats: ... frames"
- [ ] When showing RED object: Terminal shows "Detected colors: ✓ red"
- [ ] Bounding boxes appear around colored objects
- [ ] Labels show color names

**Test with colors**:

- [ ] Red object detected
- [ ] Green object detected
- [ ] Blue object detected
- [ ] Yellow object detected

**Result**: \***\*\_\_\_\*\***

---

### Test 7: Disease Detection (Full System)

```bash
# Pi Terminal 1
ros2 launch rpi_camera_package camera_compressed.launch.py pi

# Laptop Terminal 1
ros2 launch rpi_camera_package disease_detection.launch.py laptop

# Laptop Terminal 2 (monitor results)
ros2 topic echo /disease_detection/result
```

**Expected**:

- [ ] Laptop node starts with "Model loaded successfully"
- [ ] OpenCV window opens showing camera feed
- [ ] Stats printed: "📊 Stats: X frames, Y inferences"
- [ ] When showing HEALTHY leaf: "Disease: Healthy | Confidence: XX.X%"
- [ ] When showing DISEASED leaf: "Disease: Early_blight | Confidence: XX.X%" or "Disease: Late_blight | Confidence: XX.X%"
- [ ] Colored banner on image matches disease status (green=healthy, orange=early, red=late)

**Test with leaves**:

- [ ] Healthy potato leaf detected correctly
- [ ] Diseased leaf detected correctly (if available)
- [ ] Confidence scores shown (0-100%)

**Result**: \***\*\_\_\_\*\***

---

### Test 8: Full Processing (Both Detection Systems)

```bash
# Pi Terminal 1
ros2 launch rpi_camera_package camera_compressed.launch.py pi

# Laptop Terminal 1
ros2 launch rpi_camera_package full_processing.launch.py laptop
```

**Expected**:

- [ ] Both color detection and disease detection nodes start
- [ ] Two separate OpenCV windows open
- [ ] Both processing streams work simultaneously
- [ ] No significant performance degradation
- [ ] CPU usage <80% on laptop

**Result**: \***\*\_\_\_\*\***

---

## 📊 Performance Tests

### Frame Rate

```bash
# With camera running
ros2 topic hz /camera/image_raw/compressed
```

**Expected**: 20-30 Hz  
**Actual**: **\_\_\_** Hz

### Latency

```bash
ros2 topic delay /camera/image_raw/compressed
```

**Expected**: 50-150ms  
**Actual**: **\_\_\_** ms

### Bandwidth (on Pi)

```bash
# Install if needed: sudo apt install iftop
sudo iftop -i wlan0  # or eth0
```

**Expected**: ~1-3 Mbps  
**Actual**: **\_\_\_** Mbps

---

## 🔧 Parameter Tests

### Test Different JPEG Quality

```bash
# Pi - try different qualities
ros2 launch rpi_camera_package camera_compressed.launch.py pi jpeg_quality:=60
ros2 launch rpi_camera_package camera_compressed.launch.py pi jpeg_quality:=90
```

- [ ] Lower quality (60) = smaller bandwidth
- [ ] Higher quality (90) = better image quality

### Test Display Toggle (Color Detection)

```bash
# Laptop - disable display
ros2 launch rpi_camera_package color_detection.launch.py laptop display:=false
```

- [ ] Runs without OpenCV window
- [ ] Still logs detections to terminal

### Test Min Area Adjustment (Color Detection)

```bash
# Laptop - smaller threshold
ros2 launch rpi_camera_package color_detection.launch.py laptop min_area:=500
```

- [ ] Detects smaller colored objects

### Test Inference Rate (Disease Detection)

```bash
# Laptop - different inference rates
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=0.5
ros2 launch rpi_camera_package disease_detection.launch.py laptop inference_rate:=2.0
```

- [ ] 0.5 Hz = inference every 2 seconds (lower CPU)
- [ ] 2.0 Hz = inference twice per second (higher CPU)

### Test Confidence Threshold (Disease Detection)

```bash
# Laptop - require 50% confidence minimum
ros2 launch rpi_camera_package disease_detection.launch.py laptop confidence_threshold:=0.5
```

- [ ] Only reports detections with >50% confidence

---

## 🐛 Common Issues

### Camera Not Found

- [ ] Checked: Legacy camera enabled?
- [ ] Checked: Rebooted after enabling?
- [ ] Checked: Cable connected properly?
- [ ] Checked: `vcgencmd get_camera` shows detected=1?

### No Network Discovery

- [ ] Checked: Same ROS_DOMAIN_ID?
- [ ] Checked: ROS_LOCALHOST_ONLY=0?
- [ ] Checked: Firewall not blocking?
- [ ] Checked: Both on same subnet?
- [ ] Tried: `ros2 daemon stop && ros2 daemon start`?

### Low Frame Rate

- [ ] Using compressed launch file?
- [ ] Network quality good (WiFi 5GHz or Ethernet)?
- [ ] CPU usage normal (<50% on Pi)?

### Detection Not Working

- [ ] Images reaching laptop? `ros2 topic hz /camera/image_raw/compressed`
- [ ] Objects large enough? (>1000 pixels)
- [ ] Colors in detectable range?
- [ ] Good lighting?

### Disease Detection Issues

- [ ] PyTorch installed? `python3 -c "import torch"`
- [ ] Model file exists? `ls ~/ros2_ws/install/rpi_camera_package/share/rpi_camera_package/models/`
- [ ] Using potato leaves (not other plants)?
- [ ] Leaf fills 50-80% of frame?
- [ ] Consistent lighting?

---

## ✅ Final Checklist

- [ ] All hardware tests pass
- [ ] Network communication works
- [ ] Color detection works for all 4 colors
- [ ] Disease detection works for potato leaves
- [ ] ML model loads without errors
- [ ] Performance meets expectations (20+ FPS camera, 1 Hz inference)
- [ ] No errors in logs
- [ ] Display windows show detections
- [ ] Both processing modes can run simultaneously
- [ ] Documentation reviewed

---

## 📝 Notes

Add any observations, issues, or modifications here:

```
Date tested: __________
Pi IP: __________
Laptop IP: __________
Issues found:


Solutions applied:


```

---

**Status**:

- [ ] All tests passed - Ready for deployment
- [ ] Some issues found - See notes above
- [ ] Major issues - Needs debugging

---

**Tested by**: \***\*\_\_\_\*\***  
**Date**: \***\*\_\_\_\*\***  
**Signature**: \***\*\_\_\_\*\***
