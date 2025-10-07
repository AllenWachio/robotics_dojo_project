# Camera Servo Angle Configuration Guide
**Date:** October 7, 2025  
**Hardware:** Pi Camera Module with Servo Motor (PWM Pin 0)  
**Status:** ✅ CONFIGURED

---

## 📐 Servo Angle Reference System

### **Baseline: 50° = Straight Forward**

```
      Camera Angles (Side View)
      
      55° ↗  (50° + 5°)  - Angled up slightly
       |
      50° →  STRAIGHT FORWARD (Baseline) ←
       |
      45° ↘  (50° - 5°)  - Angled down slightly
```

---

## 🎯 Angle Usage in Mission

### **Current Configurations:**

| Task | Angle | Calculation | Purpose |
|------|-------|-------------|---------|
| **Disease Detection** | **45°** | 50° - 5° | Angled down to view plant leaf display |
| **Straight Forward** | **50°** | Baseline | Default/neutral camera position |
| **Elevated Viewing** | **55°** | 50° + 5° | Angled up (reserved for future use) |

---

## 🤖 Implementation Details

### **1. Default Servo Angle (sensor_behaviors.py)**

```python
# File: sensor_behaviors.py (line 690)
class ActivateCameraServo(py_trees.behaviour.Behaviour):
    """
    Servo angle reference:
    - 50° = Straight forward (baseline)
    - 45° = Angled down slightly (50° - 5°) - for disease detection
    - 55° = Angled up slightly (50° + 5°) - for elevated viewing
    """
    
    def __init__(self, name="ActivateCameraServo", target_angle=50):  # Default: 50° straight
        super().__init__(name)
        self.target_angle = target_angle
        # ...
```

### **2. Disease Detection Usage (cube_delivery_mission.py)**

```python
# File: cube_delivery_mission.py (line 190)

# Phase 0, Step 4: Adjust camera for disease detection
# 45° = 50° (straight) - 5° (angled down for plant viewing)
adjust_camera = ActivateCameraServo(
    "AdjustCameraForPlant", 
    target_angle=45  # Angled down to see plant display
)
```

---

## 📊 Complete Mission Camera Angles

### **Phase 0: Disease Detection**

```
Step 0.1: Move to plant display
Step 0.2: Turn 90° left (face plant)
Step 0.3: Stop and stabilize
Step 0.4: Adjust camera servo to 45° (50° - 5°) ← CAMERA ANGLE SET
         └─ Angled down slightly to view leaf on display
Step 0.5: Wait for potato disease detection
         └─ Camera captures image at 45° angle
         └─ Inference node processes image
Step 0.6: Return to origin
```

### **Phase 1-11: Cube Delivery**

**Note:** During cube delivery, camera is used for color detection but servo angle is **NOT** explicitly set. The camera uses:
- Either the **previous angle** (45° from disease detection), or
- **Default angle** (50° straight forward if servo was reset)

**Future Enhancement Opportunity:**
```python
# Could add at pickup (after turn, before color reading):
reset_camera = ActivateCameraServo("ResetCameraForPickup", target_angle=50)

# Or at delivery (for cube monitoring):
adjust_for_delivery = ActivateCameraServo("AdjustForDelivery", target_angle=55)
```

---

## 🔧 Hardware Configuration

### **Servo Motor Specifications:**

```
Type: Standard PWM Servo
Control Pin: PWM Pin 0 (via Arduino Mega)
Range: 0° - 180° (physical limits)
Working Range: 45° - 55° (mission use)
Baseline: 50° (straight forward)
Topic: /camera_servo/command (std_msgs/Int32)
Feedback: /camera_servo/angle (std_msgs/Int32)
```

### **Arduino Integration:**

```cpp
// Arduino Mega Servo Control
#include <Servo.h>

Servo cameraServo;
const int SERVO_PIN = 9;  // PWM pin
const int STRAIGHT_FORWARD = 50;  // Baseline angle

void setup() {
    cameraServo.attach(SERVO_PIN);
    cameraServo.write(STRAIGHT_FORWARD);  // Initialize to straight
}

void cameraServoCallback(int angle) {
    // Constrain to safe range
    angle = constrain(angle, 30, 70);  // Safety limits
    cameraServo.write(angle);
}
```

---

## 📷 Camera View Angles

### **Visual Reference:**

```
Top View (Looking Down):
        Robot
        ╔═══╗
        ║   ║
        ║ 📷║  ← Camera mounted on front
        ╚═══╝
        
Side View (Camera Angles):

55° (Up):
        ↗📷
       /
      ╔═══╗

50° (Straight):
        →📷
        ╔═══╗

45° (Down):
        ╔═══╗
         \
          ↘📷
```

### **Field of View at Different Angles:**

```
45° (Disease Detection):
├─ View: Angled down
├─ Target: Plant leaf display at robot's front-left
├─ Distance: ~0.5m from camera
└─ Purpose: Capture clear image of leaf for inference

50° (Straight Forward):
├─ View: Level with robot
├─ Target: Objects at robot's height level
├─ Distance: 0.5m - 2m effective range
└─ Purpose: Default/neutral position

55° (Elevated):
├─ View: Angled up slightly
├─ Target: Higher objects or elevated surfaces
├─ Distance: Variable
└─ Purpose: Reserved for future use
```

---

## 🧪 Testing Camera Angles

### **Test 1: Manual Servo Control**

```bash
# Test straight forward (50°)
ros2 topic pub --once /camera_servo/command std_msgs/Int32 "data: 50"

# Test disease detection angle (45°)
ros2 topic pub --once /camera_servo/command std_msgs/Int32 "data: 45"

# Test elevated angle (55°)
ros2 topic pub --once /camera_servo/command std_msgs/Int32 "data: 55"

# Monitor feedback
ros2 topic echo /camera_servo/angle
```

### **Test 2: Verify Camera View**

```bash
# Terminal 1: Launch camera node
ros2 run rpi_camera_package rpicam_node

# Terminal 2: Set angle and view image
ros2 topic pub --once /camera_servo/command std_msgs/Int32 "data: 45"

# Terminal 3: Check image topic
ros2 topic echo /camera/image_raw
# Or use rqt_image_view:
rqt_image_view
```

### **Test 3: Behavior Tree Test**

```python
# test_camera_angles.py
from sensor_behaviors import ActivateCameraServo

def test_angles():
    # Test 45° (disease detection)
    servo_45 = ActivateCameraServo("Test45", target_angle=45)
    
    # Test 50° (straight)
    servo_50 = ActivateCameraServo("Test50", target_angle=50)
    
    # Test 55° (elevated)
    servo_55 = ActivateCameraServo("Test55", target_angle=55)
```

---

## 🔄 Angle Adjustment Guidelines

### **When to Use Each Angle:**

#### **45° (50° - 5°) - Angled Down:**
✅ **Use for:**
- Disease detection (viewing plant display)
- Reading objects on ground/low surfaces
- Downward-facing color detection

❌ **Don't use for:**
- Forward navigation (blocks forward view)
- Elevated object detection

#### **50° (Baseline) - Straight Forward:**
✅ **Use for:**
- Default/neutral position
- Forward navigation assistance
- Object detection at robot height
- General purpose viewing

❌ **Don't use for:**
- Viewing objects significantly above/below robot level

#### **55° (50° + 5°) - Angled Up:**
✅ **Use for:**
- Elevated object detection
- Viewing higher surfaces
- Future extensions (shelf scanning, etc.)

❌ **Don't use for:**
- Ground-level object detection
- Disease detection

---

## ⚙️ Configuration Changes

### **To Adjust Angles:**

#### **Option 1: Change Default Angle**
```python
# In sensor_behaviors.py (line 690):
def __init__(self, name="ActivateCameraServo", target_angle=50):  # Change default here
```

#### **Option 2: Change Disease Detection Angle**
```python
# In cube_delivery_mission.py (line 190):
adjust_camera = ActivateCameraServo(
    "AdjustCameraForPlant", 
    target_angle=45  # Adjust this value (45-55° recommended)
)
```

#### **Option 3: Add Angle for Cube Delivery**
```python
# Add after turn_for_loading in cube_delivery_mission.py:
adjust_camera_pickup = ActivateCameraServo(
    "AdjustCameraForPickup", 
    target_angle=50  # Straight forward for color detection
)

# Add to tree sequence:
root.add_children([
    # ...
    turn_for_loading,
    adjust_camera_pickup,  # ← NEW
    read_color,
    # ...
])
```

---

## 📊 Angle Tolerance

### **Servo Feedback Tolerance:**

```python
# From sensor_behaviors.py (line 744):
if error <= 5:  # Within 5° tolerance
    return SUCCESS
```

**Meaning:**
- Target: 45° → Acceptable: 40° - 50°
- Target: 50° → Acceptable: 45° - 55°
- Target: 55° → Acceptable: 50° - 60°

This tolerance accounts for:
- Mechanical servo accuracy (±2-3°)
- PWM signal variation
- Physical servo limitations

---

## 🚨 Safety Limits

### **Recommended Angle Range:**

```python
# Safe operating range for mission
MIN_ANGLE = 40°  # Don't go below (might hit robot body)
MAX_ANGLE = 60°  # Don't go above (might strain servo)

# Mission working range
WORKING_MIN = 45°  # Lowest angle used (disease detection)
WORKING_MAX = 55°  # Highest angle used (future use)
BASELINE = 50°     # Default straight forward
```

### **Physical Constraints:**
- **Below 40°:** Camera may point too far down, hitting robot chassis
- **Above 60°:** Camera may point too far up, servo might strain
- **45° - 55° range:** Safe, tested, reliable operation

---

## 📝 Files Modified

| File | Changes | Lines |
|------|---------|-------|
| `sensor_behaviors.py` | Updated default angle 45° → 50° | 690 |
| `sensor_behaviors.py` | Added angle reference documentation | 681-688 |
| `cube_delivery_mission.py` | Added comment for 45° angle usage | 188-190 |
| `cube_delivery_mission.py` | Updated console output with angle info | 302 |
| `CAMERA_SERVO_ANGLES.md` | NEW - This documentation | All |

---

## 🎓 Quick Reference Card

```
╔════════════════════════════════════════════════════╗
║      PI CAMERA SERVO ANGLE QUICK REFERENCE        ║
╠════════════════════════════════════════════════════╣
║                                                    ║
║  BASELINE: 50° = STRAIGHT FORWARD                  ║
║                                                    ║
║  ANGLES:                                           ║
║  ├─ 45° (50° - 5°) = Disease Detection            ║
║  ├─ 50° (Baseline) = Straight Forward (Default)   ║
║  └─ 55° (50° + 5°) = Elevated Viewing (Future)    ║
║                                                    ║
║  USAGE:                                            ║
║  └─ Phase 0, Step 4: 45° (angled down for plant)  ║
║                                                    ║
║  TOPIC: /camera_servo/command (Int32)              ║
║  FEEDBACK: /camera_servo/angle (Int32)             ║
║                                                    ║
║  TOLERANCE: ±5° acceptable                         ║
║  SAFE RANGE: 40° - 60°                             ║
║                                                    ║
╚════════════════════════════════════════════════════╝
```

---

## 🔗 Related Documentation

- **Complete Mission Guide:** `COMPLETE_MISSION_GUIDE.md`
- **Disease Detection:** `DISEASE_DETECTION_QUICKSTART.md`
- **Pi Camera Setup:** `../rpi_camera_package/README.md`
- **Sensor Behaviors:** `sensor_behaviors.py`

---

**Summary:** The camera servo uses **50° as straight forward** (baseline). Disease detection uses **45°** (50° - 5°, angled down). Future tasks can use **55°** (50° + 5°, angled up). All angles are within ±5° of the 50° baseline for consistency. 📷✅

**Implementation Date:** October 7, 2025  
**Hardware:** Pi Camera + PWM Servo (Pin 0)  
**Status:** ✅ Configured and Documented
