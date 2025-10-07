# Quick Reference: Manual Loading Timing
**Date:** October 7, 2025

---

## ⏱️ **TOTAL TIME FOR MANUAL INTERVENTION: 18 SECONDS**

---

## 📊 **Timing Breakdown**

```
Pickup Sequence at Point 1:

┌─────────────────────────────────────────────────┐
│ 1. Robot arrives at pickup location        ✓   │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ 2. StopRobot: Stabilize for 3 SECONDS      ⏱️   │
│    [Robot is stationary, preparing sensors]     │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ 3. ReadColorSensor: Wait 15 SECONDS        ⏱️   │
│    ├─ Turn on LED                               │
│    ├─ Attempt to read cube color                │
│    ├─ If detected → Continue (automatic)        │
│    └─ If timeout → Manual loading mode          │
│                                                  │
│    🤚 YOU HAVE 15 SECONDS TO LOAD CUBE HERE!    │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ 4. Continue to delivery location           ✓   │
└─────────────────────────────────────────────────┘

═══════════════════════════════════════════════════
TOTAL MANUAL INTERVENTION WINDOW:
3 seconds (stabilization) + 15 seconds (sensor) = 18 SECONDS
═══════════════════════════════════════════════════
```

---

## 🎯 **When to Load the Cube**

### **Option 1: Automatic (Sensor Works)**
```
✓ Robot stops at pickup
✓ Cube already on platform
✓ Sensor detects color within 1-2 seconds
✓ Robot continues immediately to delivery
```

### **Option 2: Manual Loading (Sensor Fails/No Cube)**
```
✓ Robot stops at pickup
⏱️  Wait 3 seconds (stabilization)
⏱️  Sensor attempts detection for 15 seconds
⚠️  TIMEOUT after 15 seconds
🤚 YOU: Place cube on robot NOW!
   └─ You have the FULL 18 seconds to place cube
   └─ Robot will continue as soon as you're done
✓ Robot continues to delivery
```

---

## 📢 **Console Messages Timeline**

### **At Pickup Location:**

```bash
[Time 0:00] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] MoveToPosition: ✓ Reached pickup location

[Time 0:00-0:03] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] StopRobot: Stopping for 3.0 seconds...
[INFO] StopRobot: ✓ Stop complete

[Time 0:03-0:18] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] ReadColorSensor: LED ON, reading color sensor...

👉 IF SENSOR DETECTS CUBE (within 1-2 seconds):
   [INFO] ReadColorSensor: ✓ Detected color = RED
   [INFO] MonitorCameraForColor: Monitoring camera for RED...
   ✅ Continues to delivery immediately

👉 IF NO CUBE DETECTED (after 15 seconds):
   [WARNING] ReadColorSensor: ⚠️  Timeout waiting for color reading (15s)
   [INFO] ReadColorSensor: 🤚 Manual intervention expected
   [INFO] MonitorCameraForColor: ℹ️  Color is 'unknown' - skipping monitoring
   ✅ Continues to delivery (you have loaded cube)
```

---

## ⚙️ **Configuration Values**

```python
# File: sensor_behaviors.py
class ReadColorSensor:
    def __init__(self):
        self.timeout = 15.0  # 15 seconds for manual loading

# File: cube_delivery_mission.py
stop_at_point1 = StopRobot("StopAtPoint1", duration=3.0)  # 3 seconds stabilization
```

---

## 🔧 **How to Adjust Timing** (If Needed)

### **To Give More Time:**

**Option 1: Increase sensor timeout**
```python
# In sensor_behaviors.py line 37:
self.timeout = 20.0  # Change from 15 to 20 seconds
```

**Option 2: Increase stop duration**
```python
# In cube_delivery_mission.py line 216:
stop_at_point1 = StopRobot("StopAtPoint1", duration=5.0)  # Change from 3 to 5 seconds
```

### **To Speed Up (Automatic Detection Only):**
```python
# In sensor_behaviors.py line 37:
self.timeout = 10.0  # Reduce to 10 seconds (if sensor is fast)

# In cube_delivery_mission.py line 216:
stop_at_point1 = StopRobot("StopAtPoint1", duration=1.0)  # Reduce to 1 second
```

---

## 🚨 **Important Notes**

1. **18 seconds is TOTAL maximum time**
   - Sensor starts reading after 3 second stabilization
   - You have up to 15 seconds during sensor reading phase
   - If you load earlier, robot continues immediately

2. **Robot will NOT wait forever**
   - After 18 seconds total, it will continue regardless
   - Make sure cube is loaded within this time
   - Mission continues even if cube not loaded (you can add it at delivery)

3. **Sensor Detection is Faster**
   - If sensor detects cube (automatic mode): ~1-2 seconds
   - If manual loading needed: Full 18 seconds available
   - Robot adapts based on sensor success/failure

---

## ✅ **Quick Checklist**

Before running mission:
- [ ] Understand you have **18 seconds** to manually load
- [ ] Know when to intervene (watch for WARNING message)
- [ ] Practice placing cube on robot platform
- [ ] Verify cube is secure before robot moves
- [ ] Monitor console for timing messages

---

**Bottom Line:** You have a comfortable **18 second window** (3s stabilization + 15s sensor timeout) to manually load the cube if the sensor doesn't detect it automatically. The robot will wait patiently! 🤚⏱️

---

**See also:**
- `MANUAL_INTERVENTION_SUPPORT.md` - Complete manual loading guide
- `COMPLETE_MISSION_GUIDE.md` - Full mission documentation
- `cube_delivery_mission.py` - Mission configuration
