# 🚀 Quick Reference Card - Disease Detection

## ⚡ Easiest Way: Shell Scripts (Recommended)

### **Pi (Camera)**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/pi
./run_camera.sh
```

### **Laptop - Color Detection**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
./run_color_detection.sh
```

### **Laptop - Disease Detection**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
./run_disease_detection.sh
```

### **Laptop - Both (Color + Disease)**

```bash
cd ~/ros2_ws/src/rpi_camera_package/scripts/laptop
./run_full_processing.sh
```

> ✅ **Scripts include all pre-flight checks and error handling!**

---

## ⚡ Alternative: Direct Launch Commands

### **Pi (Camera)**

```bash
ros2 launch rpi_camera_package camera_compressed.launch.py pi
```

### **Laptop - Disease Detection Only**

```bash
ros2 launch rpi_camera_package disease_detection.launch.py laptop
```

### **Laptop - Both Color + Disease**

```bash
ros2 launch rpi_camera_package full_processing.launch.py laptop
```

---

## 📊 Monitor Results

```bash
# Disease detection results
ros2 topic echo /disease_detection/result

# Color detection results
ros2 topic echo /color_detection/processed_image

# Camera feed rate
ros2 topic hz /camera/image_raw/compressed
```

---

## 🔧 Common Parameters

### **Inference Rate** (CPU optimization)

```bash
# Slow (0.5 Hz = every 2 seconds)
inference_rate:=0.5

# Default (1 Hz = once per second)
inference_rate:=1.0

# Fast (2 Hz = twice per second)
inference_rate:=2.0
```

### **Confidence Threshold** (Filter results)

```bash
# Show all (0%)
confidence_threshold:=0.0

# Show if >50% confident
confidence_threshold:=0.5

# Show if >80% confident
confidence_threshold:=0.8
```

### **Headless Mode** (No display)

```bash
display:=false
```

---

## 🎯 Why Use Shell Scripts?

✅ **No parameters to remember** - All pre-configured  
✅ **Pre-flight checks** - Verifies everything before launch  
✅ **Colored output** - Easy to read error messages  
✅ **Automatic setup** - Sources workspace, sets network config  
✅ **Beginner-friendly** - Clear error messages with solutions

See `scripts/README.md` for full documentation.

---

## 📖 Documentation

| File                                    | Purpose                |
| --------------------------------------- | ---------------------- |
| **README.md**                           | Main documentation     |
| **DISEASE_DETECTION.md**                | ML usage guide ⭐      |
| **QUICKSTART.md**                       | Quick commands         |
| **scripts/README.md**                   | Shell scripts guide ⭐ |
| **TESTING_CHECKLIST.md**                | Verification steps     |
| **DISEASE_DETECTION_IMPLEMENTATION.md** | Implementation details |

---

## 🌱 Disease Classes

| Class            | Description         | Color Code |
| ---------------- | ------------------- | ---------- |
| **Healthy**      | Normal potato leaf  | 🟢 Green   |
| **Early_blight** | Early stage disease | 🟠 Orange  |
| **Late_blight**  | Late stage disease  | 🔴 Red     |

---

## 🐛 Quick Fixes

### **Model not found?**

```bash
ls ~/ros2_ws/install/rpi_camera_package/share/rpi_camera_package/models/
colcon build --packages-select rpi_camera_package
```

### **No images?**

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic hz /camera/image_raw/compressed
```

### **High CPU?**

```bash
# Reduce inference rate
inference_rate:=0.5
```

---

## 📈 Expected Performance

- **Camera**: 20-30 FPS
- **ML Inference**: 1 Hz (default)
- **CPU (Laptop)**: 30-50% per node
- **Bandwidth**: 1-3 Mbps (compressed)

---

**Last Updated**: October 5, 2025
