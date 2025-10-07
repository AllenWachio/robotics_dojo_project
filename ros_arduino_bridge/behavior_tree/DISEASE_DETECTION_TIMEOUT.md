# Disease Detection Timeout Configuration
**Date:** October 7, 2025  
**Feature:** Non-blocking disease detection with graceful timeout  
**Status:** ✅ IMPLEMENTED

---

## 🕐 Overview

The disease detection phase now includes a **12-second timeout**. If the inference node doesn't provide a result within 12 seconds, the robot will:

1. ⚠️ Log a timeout warning
2. 📝 Store 'timeout' in blackboard
3. 🚀 **Continue with the mission** (move on to cube delivery)

**Key Point:** Disease detection failure does NOT stop the mission!

---

## ⚙️ Implementation Details

### **1. Behavior Timeout (sensor_behaviors.py)**

```python
class WaitForDiseaseDetection(py_trees.behaviour.Behaviour):
    """
    Waits for potato disease detection result from /inference_result topic.
    
    Timeout Behavior:
    - If result received: Returns SUCCESS with disease name
    - If timeout (12s): Returns FAILURE with 'timeout' stored
    """
    
    def __init__(self, name="WaitForDiseaseDetection", timeout=10.0):
        self.timeout = timeout  # 12 seconds in mission
        # ...
    
    def update(self):
        # Check if result received
        if self.detection_result is not None:
            self.blackboard.set('disease_detection_result', self.detection_result)
            return py_trees.common.Status.SUCCESS
        
        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed >= self.timeout:
            self.logger.warning(
                f"{self.name}: ⚠️  Detection timeout ({self.timeout}s) "
                "- cancelling and moving on"
            )
            self.blackboard.set('disease_detection_result', 'timeout')
            return py_trees.common.Status.FAILURE  # ← Returns FAILURE on timeout
        
        return py_trees.common.Status.RUNNING
```

**File:** `sensor_behaviors.py` (lines 817-834)

---

### **2. Mission Integration (cube_delivery_mission.py)**

```python
# Step 0.5: Wait for disease detection (12 seconds max - then cancel and move on)
# Wrap in decorators to make timeout non-fatal
detect_disease = py_trees.decorators.Timeout(
    name="DiseaseDetectionTimeout",
    child=WaitForDiseaseDetection("DetectPotatoDisease", timeout=12.0),
    duration=12.5  # Slightly longer than behavior timeout
)

# FailureIsSuccess: Converts FAILURE → SUCCESS to continue mission
detect_disease = py_trees.decorators.FailureIsSuccess(
    name="OptionalDiseaseDetection",
    child=detect_disease
)
```

**File:** `cube_delivery_mission.py` (lines 193-204)

---

## 🎯 Behavior Tree Structure

```
Phase 0: Disease Detection Sequence
│
├─ 0.1: Move to plant display
├─ 0.2: Turn 90° left
├─ 0.3: Stop and stabilize
├─ 0.4: Adjust camera servo (45°)
├─ 0.5: Disease detection (12s timeout) ← WRAPPED IN DECORATORS
│      ├─ Inner: Timeout decorator (12.5s outer limit)
│      └─ Outer: FailureIsSuccess (converts FAILURE → SUCCESS)
└─ 0.6: Return to origin

If detection succeeds in <12s:
  ✅ Result stored → Continue to 0.6

If detection times out at 12s:
  ⚠️  'timeout' stored → FAILURE → FailureIsSuccess converts to SUCCESS
  → Continue to 0.6 (mission proceeds normally)
```

---

## 🔧 Decorator Explanation

### **py_trees.decorators.Timeout**

```python
Timeout(child=..., duration=12.5)
```

**Purpose:** Hard deadline for child behavior  
**Behavior:**
- If child completes in <12.5s → Returns child's status (SUCCESS/FAILURE)
- If child still RUNNING at 12.5s → Returns FAILURE

**Why 12.5s vs 12s?**
- Behavior has 12s internal timeout
- Decorator gives 0.5s buffer for cleanup/logging

---

### **py_trees.decorators.FailureIsSuccess**

```python
FailureIsSuccess(child=...)
```

**Purpose:** Convert FAILURE to SUCCESS  
**Behavior:**
- Child returns SUCCESS → Returns SUCCESS
- Child returns FAILURE → **Converts to SUCCESS** ← KEY FEATURE
- Child returns RUNNING → Returns RUNNING

**Why use this?**
- Makes disease detection **optional**
- If timeout occurs (FAILURE), mission continues
- No interruption to cube delivery phase

---

## 📊 Possible Outcomes

### **Outcome 1: Detection Successful (< 12 seconds)**

```
Timeline:
0.0s → Disease detection starts
3.5s → Inference node publishes result
3.5s → WaitForDiseaseDetection receives result
3.5s → Returns SUCCESS
3.5s → FailureIsSuccess passes SUCCESS through
3.5s → Mission continues to step 0.6

Blackboard:
  disease_detection_result = "Early Blight" (or "Late Blight", "Healthy")

Console:
  ✓ Detection complete: Early Blight
```

---

### **Outcome 2: Detection Timeout (12 seconds)**

```
Timeline:
0.0s  → Disease detection starts
12.0s → WaitForDiseaseDetection timeout reached
12.0s → Returns FAILURE
12.0s → FailureIsSuccess converts FAILURE → SUCCESS
12.0s → Mission continues to step 0.6

Blackboard:
  disease_detection_result = "timeout"

Console:
  ⚠️  Detection timeout (12.0s) - cancelling and moving on
  (Mission proceeds normally)
```

---

### **Outcome 3: Outer Timeout (12.5 seconds, rare)**

```
Timeline:
0.0s  → Disease detection starts
12.5s → Outer Timeout decorator triggers
12.5s → Returns FAILURE (behavior didn't complete)
12.5s → FailureIsSuccess converts FAILURE → SUCCESS
12.5s → Mission continues to step 0.6

Blackboard:
  disease_detection_result = "timeout" (from behavior)
  OR may not be set if behavior was killed mid-execution

Console:
  (Timeout message from decorator)
  (Mission proceeds normally)
```

---

## 🧪 Testing the Timeout

### **Test 1: Normal Operation (Inference Working)**

```bash
# Terminal 1: Launch inference node
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node

# Terminal 2: Run mission
python3 cube_delivery_mission.py

# Expected:
# - Robot moves to plant display
# - Camera adjusts to 45°
# - Inference processes image in ~2-5 seconds
# - Result received: "Early Blight" / "Late Blight" / "Healthy"
# - Mission continues
```

---

### **Test 2: Timeout Scenario (Inference NOT Running)**

```bash
# Terminal 1: DON'T launch inference node (simulate failure)

# Terminal 2: Run mission
python3 cube_delivery_mission.py

# Expected:
# - Robot moves to plant display
# - Camera adjusts to 45°
# - Waits for 12 seconds (no inference result)
# - Timeout warning: "⚠️  Detection timeout (12.0s) - cancelling and moving on"
# - Mission continues to step 0.6 (return to origin)
# - Cube delivery proceeds normally
```

---

### **Test 3: Manual Timeout Trigger**

```python
# Create test script: test_timeout.py
from sensor_behaviors import WaitForDiseaseDetection
import time

# Test with very short timeout
behavior = WaitForDiseaseDetection("Test", timeout=3.0)
# ... (setup ROS2 node, etc.)

# Wait 5 seconds without publishing result
time.sleep(5)

# Check that behavior returned FAILURE
status = behavior.update()
assert status == py_trees.common.Status.FAILURE
```

---

## 🔧 Configuration Options

### **Change Timeout Duration:**

**Option 1: Increase to 20 seconds**
```python
# In cube_delivery_mission.py (line 198):
detect_disease = py_trees.decorators.Timeout(
    name="DiseaseDetectionTimeout",
    child=WaitForDiseaseDetection("DetectPotatoDisease", timeout=20.0),  # ← Change
    duration=20.5  # ← Also update outer timeout
)
```

**Option 2: Decrease to 8 seconds**
```python
detect_disease = py_trees.decorators.Timeout(
    name="DiseaseDetectionTimeout",
    child=WaitForDiseaseDetection("DetectPotatoDisease", timeout=8.0),  # ← Change
    duration=8.5  # ← Also update outer timeout
)
```

---

### **Make Detection Mandatory (Remove Timeout Protection):**

```python
# Remove FailureIsSuccess decorator
detect_disease = py_trees.decorators.Timeout(
    name="DiseaseDetectionTimeout",
    child=WaitForDiseaseDetection("DetectPotatoDisease", timeout=12.0),
    duration=12.5
)
# Now timeout will STOP the mission (Sequence fails)
```

---

### **Remove Timeout Entirely:**

```python
# Just use behavior without decorators
detect_disease = WaitForDiseaseDetection("DetectPotatoDisease", timeout=999999)
# Robot will wait indefinitely for result
```

---

## 📈 Timeout Behavior Comparison

| Configuration | Timeout Occurs | Mission Continues? | Use Case |
|---------------|----------------|-------------------|----------|
| **Current (12s + FailureIsSuccess)** | ⚠️ Warning logged | ✅ YES | **Best for demos/testing** |
| **No FailureIsSuccess decorator** | ❌ Mission stops | ❌ NO | Critical detection required |
| **No timeout (999999s)** | Never | ✅ (waits forever) | Debugging inference issues |
| **Very short (3s)** | ⚠️ Almost always | ✅ YES | Fast testing cycles |

---

## 📝 Blackboard Data

### **After Successful Detection:**

```python
blackboard.get('disease_detection_result')
# Returns: "Early Blight" or "Late Blight" or "Healthy"
```

### **After Timeout:**

```python
blackboard.get('disease_detection_result')
# Returns: "timeout"
```

### **Checking in Later Behaviors:**

```python
# Example: Log result at mission completion
result = blackboard.get('disease_detection_result')
if result == 'timeout':
    print("⚠️  Disease detection timed out - result not available")
elif result:
    print(f"✅ Disease detected: {result}")
```

---

## 🚨 Troubleshooting

### **Issue 1: Detection Always Times Out**

**Symptoms:**
- Always get timeout warning after 12s
- Never see "Detection complete" message

**Possible Causes:**
1. Inference node not running
2. Topic mismatch (wrong topic name)
3. Camera not publishing images
4. Model not loaded

**Solutions:**
```bash
# Check if inference node is running
ros2 node list | grep potato

# Check if topic exists and has data
ros2 topic list | grep inference
ros2 topic echo /inference_result

# Check camera feed
ros2 topic echo /camera/image_raw

# Launch inference manually
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node
```

---

### **Issue 2: Detection Too Slow (> 12s)**

**Symptoms:**
- Detection would succeed, but times out first
- Inference takes 15-20 seconds

**Solutions:**
```python
# Option A: Increase timeout
detect_disease = WaitForDiseaseDetection("DetectPotatoDisease", timeout=25.0)

# Option B: Optimize inference node
# - Use GPU instead of CPU
# - Reduce image resolution
# - Use smaller model
```

---

### **Issue 3: Mission Stops Despite FailureIsSuccess**

**Symptoms:**
- Timeout occurs
- Mission stops at step 0.5

**Cause:**
- FailureIsSuccess decorator might not be applied correctly

**Solution:**
```python
# Verify both decorators are present:
detect_disease = py_trees.decorators.Timeout(...)  # Line 195
detect_disease = py_trees.decorators.FailureIsSuccess(  # Line 201
    child=detect_disease  # ← Must wrap the Timeout decorator
)
```

---

## 🎓 Quick Reference Card

```
╔════════════════════════════════════════════════════════╗
║    DISEASE DETECTION TIMEOUT QUICK REFERENCE          ║
╠════════════════════════════════════════════════════════╣
║                                                        ║
║  TIMEOUT: 12 seconds                                   ║
║  BEHAVIOR: Non-fatal (mission continues)               ║
║                                                        ║
║  IMPLEMENTATION:                                       ║
║  └─ WaitForDiseaseDetection (12s internal timeout)    ║
║     └─ Returns FAILURE on timeout                      ║
║     └─ Stores 'timeout' in blackboard                  ║
║                                                        ║
║  DECORATORS:                                           ║
║  ├─ Timeout (12.5s outer limit)                        ║
║  └─ FailureIsSuccess (converts FAILURE → SUCCESS)      ║
║                                                        ║
║  OUTCOMES:                                             ║
║  ├─ Success (<12s): Result stored, continue            ║
║  └─ Timeout (12s):  'timeout' stored, continue         ║
║                                                        ║
║  FILES MODIFIED:                                       ║
║  ├─ sensor_behaviors.py (lines 817-834)                ║
║  └─ cube_delivery_mission.py (lines 193-204)           ║
║                                                        ║
╚════════════════════════════════════════════════════════╝
```

---

## 🔗 Related Documentation

- **Complete Mission Guide:** `COMPLETE_MISSION_GUIDE.md`
- **Disease Detection Setup:** `DISEASE_DETECTION_QUICKSTART.md`
- **Camera Servo Angles:** `CAMERA_SERVO_ANGLES.md`
- **Behavior Tree Basics:** `PYTREES_INTEGRATION_GUIDE.md`

---

**Summary:** Disease detection now has a **12-second timeout**. If the inference node doesn't respond in time, the behavior returns FAILURE, which is converted to SUCCESS by the `FailureIsSuccess` decorator. The mission continues normally with 'timeout' stored in the blackboard. This prevents the robot from getting stuck waiting indefinitely for inference results. 🕐✅

**Implementation Date:** October 7, 2025  
**Timeout Duration:** 12 seconds  
**Status:** ✅ Non-fatal, mission continues on timeout
