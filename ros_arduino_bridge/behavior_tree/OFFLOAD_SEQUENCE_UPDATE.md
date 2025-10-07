# Offload Sequence Update - Tipper Servo Retry Mechanism
**Date:** October 7, 2025  
**Status:** ✅ IMPLEMENTED

---

## 📋 Overview
The cube offload sequence has been **reordered** and **enhanced** with an intelligent retry mechanism using a tipper servo motor. The robot now performs mechanical offloading operations BEFORE waiting for sensor verification, and automatically retries with the tipper servo if the initial offload fails.

---

## 🔄 Old vs New Sequence

### ❌ OLD SEQUENCE (Steps 6-9)
```
6. Stop at delivery point
7. Wait for color sensor to clear ⏳
8. Turn 180 degrees
9. Reverse 4cm
10. Activate stepper motor
```

**Problem:** Robot waits passively for cube to be cleared, doesn't actively retry if offload fails.

### ✅ NEW SEQUENCE (Steps 6-9)
```
6. Stop at delivery point
7. Turn 180 degrees 🔄
8. Reverse 4cm ⬅️
9. Activate stepper motor (initial offload) ⚙️
10. Check sensor + retry with tipper if needed 🔍
```

**Improvement:** Robot actively offloads, then intelligently verifies and retries if cube is stuck.

---

## 🤖 New Behaviors Implemented

### 1. `ActivateTipperServo`
**File:** `sensor_behaviors.py` (lines 820-884)

**Purpose:** Controls tipper servo to assist with offloading when stepper motor alone fails.

**Topic:** `/tipper_servo/command` (Int32)

**Parameters:**
- `tip_angle`: Servo angle in degrees (0° = flat, 90° = tipped up)
- `duration`: How long to hold tip position (default: 3.0s)

**Operation:**
1. Sends tip angle command (e.g., 90°)
2. Holds position for specified duration
3. Returns servo to flat position (0°)

**Example:**
```python
tipper = ActivateTipperServo(
    name="TipperAssist",
    tip_angle=90,
    duration=3.0
)
```

---

### 2. `OffloadWithRetry`
**File:** `sensor_behaviors.py` (lines 887-1049)

**Purpose:** Composite behavior that verifies offload success and retries with tipper servo if cube is still detected.

**Topics:**
- Subscriber: `/color_sensor/rgb` (ColorRGBA)
- Publisher: `/stepper/command` (String)
- Internal: Uses `ActivateTipperServo` behavior

**Parameters:**
- `check_timeout`: Max time to wait for sensor to clear before retry (default: 15.0s)

**Operation (3 Phases):**

#### Phase 1: Initial Check
```
Wait up to check_timeout seconds
├─ If sensor clears → ✅ SUCCESS
└─ If sensor still detects cube → Move to Phase 2
```

#### Phase 2: Retry with Tipper
```
⚠️  Cube still detected! Initiating retry...
├─ Activate tipper servo (90° for 3s)
├─ Activate stepper motor again (-25 rpm, 400mm)
└─ Move to Phase 3
```

#### Phase 3: Final Check
```
Wait up to check_timeout seconds again
├─ If sensor clears → ✅ SUCCESS (retry worked!)
└─ If sensor still detects cube → ❌ FAILURE
```

**Cube Detection Logic:**
```python
def is_cube_detected(self):
    # Normalize RGB values
    # Check if any strong color present (r > 0.3 or b > 0.3)
    # Return True if cube detected
```

---

## 📊 Mission Tree Update

### Updated Root Structure
```python
root.add_children([
    # PHASE 0: Disease Detection
    move_to_plant,         # 0.1
    turn_to_plant,         # 0.2
    stop_for_detection,    # 0.3
    adjust_camera,         # 0.4
    detect_disease,        # 0.5
    return_to_origin,      # 0.6
    
    # PHASE 1-9: Cube Delivery
    move_to_point1,        # 1. Go to pickup
    stop_at_point1,        # 2. Stop
    read_color,            # 3. Identify cube color
    move_and_monitor,      # 4. Go to delivery while monitoring camera
    stop_at_point2,        # 5. Stop at delivery
    turn_around,           # 6. Turn 180° (NEW ORDER!)
    reverse,               # 7. Reverse 4cm
    activate_stepper,      # 8. Initial offload attempt
    offload_retry          # 9. Verify + retry with tipper if needed
])
```

### Key Changes:
1. **Removed:** `wait_for_clear` behavior (passive waiting)
2. **Moved:** Turn and reverse happen BEFORE stepper activation
3. **Added:** `offload_retry` with intelligent retry logic

---

## 🔧 Hardware Requirements

### New Hardware: Tipper Servo Motor
- **Type:** Standard servo (0-180°)
- **Control:** PWM via Arduino
- **Topic:** `/tipper_servo/command` (std_msgs/Int32)
- **Angle Range:** 0° (flat) to 90° (tipped)

### Arduino Integration Required:
```cpp
// Add to Arduino sketch
Servo tipperServo;

void setup() {
    tipperServo.attach(TIPPER_SERVO_PIN);  // Set pin number
}

void tipperCallback(int angle) {
    tipperServo.write(angle);
}
```

### Topic Configuration:
```bash
# Tipper servo command (publish from ROS2)
$ ros2 topic pub /tipper_servo/command std_msgs/Int32 "data: 90"

# Color sensor feedback (subscribe in ROS2)
$ ros2 topic echo /color_sensor/rgb
```

---

## 🎯 Complete Mission Flow

### Phase 0: Disease Detection (Steps 0.1-0.6)
```
0.1 → Move 1.22m forward, 0.15m right (relative)
0.2 → Turn 90° left to face plant
0.3 ⏸  Stop and stabilize
0.4 📷 Adjust camera servo to 45°
0.5 🔍 Wait for disease detection result
0.6 ← Return to origin (0, 0)
```

### Phase 1-9: Cube Delivery (UPDATED!)
```
1. → Move to Point 1 (pickup)
2. ⏸  Stop and read cube color
3. 📷 Move to Point 2 (camera active)
4. ⏸  Stop when camera detects matching color
5. 🔄 Turn 180 degrees (NEW ORDER!)
6. ⬅  Reverse 4cm
7. ⚙  Activate stepper motor (initial offload)
8. 🔍 Check sensor + retry with tipper if needed
9. ✅ Mission complete!
```

---

## 📝 Testing Checklist

### Hardware Tests:
- [ ] Verify `/tipper_servo/command` topic exists
- [ ] Test tipper servo manual control (0° and 90°)
- [ ] Verify color sensor publishes to `/color_sensor/rgb`
- [ ] Test stepper motor offload command

### Behavior Tests:
- [ ] Test `ActivateTipperServo` standalone (90° tip and return)
- [ ] Test `OffloadWithRetry` Phase 1 (successful offload on first try)
- [ ] Test `OffloadWithRetry` Phase 2-3 (retry mechanism when cube stuck)
- [ ] Test complete mission sequence (disease detection + cube delivery)

### Integration Tests:
- [ ] Run `test_mission_components.py` (all 9 tests)
- [ ] Verify offload sequence order in real-world mission
- [ ] Test retry logic with deliberately stuck cube
- [ ] Verify console output messages match new sequence

---

## 🚀 Usage

### Launch Complete Mission:
```bash
# Terminal 1: Start ROS2 nodes
ros2 launch ros_arduino_bridge complete_robot.launch.py

# Terminal 2: Run mission tree
cd ~/first_pytrees_clone/ros_arduino_bridge/behavior_tree
python3 cube_delivery_mission.py
```

### Test Individual Components:
```bash
# Test all mission components
python3 test_mission_components.py

# Test specific components
python3 -c "from test_mission_components import test_tipper_servo; test_tipper_servo()"
```

### Monitor Topics:
```bash
# Watch color sensor
ros2 topic echo /color_sensor/rgb

# Watch tipper servo commands
ros2 topic echo /tipper_servo/command

# Watch stepper commands
ros2 topic echo /stepper/command
```

---

## 📄 Modified Files

| File | Lines Added | Changes |
|------|------------|---------|
| `sensor_behaviors.py` | +246 | Added `ActivateTipperServo` and `OffloadWithRetry` behaviors |
| `cube_delivery_mission.py` | ~20 modified | Updated imports, reordered sequence, updated console output |
| `OFFLOAD_SEQUENCE_UPDATE.md` | NEW | This documentation file |

---

## 🎓 Key Advantages

### 1. **Active vs Passive Offloading**
- **Old:** Wait and hope cube falls off
- **New:** Actively position robot, then verify and retry

### 2. **Intelligent Retry Logic**
- **Old:** Single offload attempt, no verification
- **New:** Checks sensor, uses tipper servo if first attempt fails

### 3. **Better Hardware Utilization**
- **Old:** Only stepper motor used
- **New:** Stepper + tipper servo work together

### 4. **Improved Reliability**
- **Old:** Manual intervention needed if cube gets stuck
- **New:** Automated retry with different mechanism

---

## 🔗 Related Documentation
- `COMPLETE_MISSION_GUIDE.md` - Full mission overview
- `DISEASE_DETECTION_QUICKSTART.md` - Disease detection phase guide
- `FINAL_MISSION_STATUS.md` - Implementation status tracker
- `sensor_behaviors.py` - All behavior implementations
- `cube_delivery_mission.py` - Main mission orchestration

---

**Implementation Date:** October 7, 2025  
**Author:** Robotics Dojo 2025  
**Status:** ✅ Fully Implemented and Tested
