# Back Right Motor (M4) Troubleshooting Guide

## Problem Statement
- **Manual serial command works**: `m 0:0:0:100` → M4 spins correctly
- **Teleop commands fail**: M4 doesn't respond during backward, left, or right commands
- **Other 3 motors work fine**: M1 (FL), M2 (FR), M3 (RL) all respond correctly

## Key Insight
Since ALL motors get the SAME PWM value when going backward (`m -255:-255:-255:-255`), but only M4 fails, this is **NOT** a calculation bug. It's a hardware, parsing, or timing issue specific to M4.

## Diagnostic Steps

### Step 1: Verify Command String Format

**Test on Raspberry Pi:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
```

**In another terminal, test each movement:**
```bash
# Test 1: Forward (all motors positive)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
# Expected log: PWM: M1(FL)=255 M2(FR)=255 M3(RL)=255 M4(RR)=255

# Test 2: Backward (all motors negative)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}"
# Expected log: PWM: M1(FL)=-255 M2(FR)=-255 M3(RL)=-255 M4(RR)=-255

# Test 3: Turn left (left negative, right positive)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
# Expected log: PWM: M1(FL)=-76 M2(FR)=76 M3(RL)=-76 M4(RR)=76

# Test 4: Turn right (left positive, right negative)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -1.0}}"
# Expected log: PWM: M1(FL)=76 M2(FR)=-76 M3(RL)=76 M4(RR)=-76
```

**Check the logs** and paste the EXACT output for each test, especially:
- What PWM values are calculated for M4?
- Is the command string correct?
- Does Arduino acknowledge the command?

---

### Step 2: Direct Arduino Serial Test

**Open serial monitor to Arduino** (baud 57600):

```
# Test M4 directly
m 0:0:0:100
# M4 should spin forward

m 0:0:0:-100
# M4 should spin backward

m 100:100:100:100
# All 4 motors should spin forward

m -100:-100:-100:-100
# All 4 motors should spin backward
```

**Compare:** Does M4 respond to **all** of these manual commands? If yes, the Arduino parsing is fine.

---

### Step 3: Check for Command Timing Issues

**Theory:** If Python sends commands too fast, Arduino buffer might overflow and corrupt M4 value (since it's parsed last).

**Test:** Add delay between commands in teleop:

```bash
# Send commands with 0.5 second delay
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}"
sleep 0.5
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}"
sleep 0.5
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}"
```

**Does M4 work now?** If yes, it's a buffer/timing issue.

---

### Step 4: Check Arduino Parsing Code

**Look for M4-specific parsing bugs** in your Arduino code:

```cpp
// Common bug pattern:
void parseMotorCommand(String cmd) {
    int colon1 = cmd.indexOf(':');
    int colon2 = cmd.indexOf(':', colon1 + 1);
    int colon3 = cmd.indexOf(':', colon2 + 1);
    
    int M1 = cmd.substring(2, colon1).toInt();
    int M2 = cmd.substring(colon1 + 1, colon2).toInt();
    int M3 = cmd.substring(colon2 + 1, colon3).toInt();
    int M4 = cmd.substring(colon3 + 1).toInt();  // ← Might capture \r\n garbage!
    
    // BETTER: Trim or explicitly specify end position
    int M4 = cmd.substring(colon3 + 1, cmd.length() - 1).toInt();  // Exclude \r
}
```

**Check:** Does Arduino code trim the command string before parsing M4?

---

### Step 5: Check Motor Driver Wiring

**Hardware issue possibilities:**
1. M4 wire loose on motor driver
2. M4 PWM pin not properly connected
3. Motor driver channel 4 damaged (unlikely if manual commands work)

**Test:**
```bash
# Swap M3 and M4 wires on motor driver
# If problem follows the motor → motor issue
# If problem follows the channel → driver issue
```

---

## Root Cause Hypotheses (Ranked)

### 1. Arduino String Parsing Bug (Most Likely)
**Symptom:** M4 value gets corrupted by trailing characters (`\r\n`)

**Evidence:**
- Manual typing works (no `\r` sent, or different timing)
- Python sends `m -255:-255:-255:-255\r`
- Arduino parses M4 as last substring → might capture garbage

**Fix:** Update Arduino code to trim command:
```cpp
String cmd = Serial.readStringUntil('\r');
cmd.trim();  // Remove any \r\n whitespace
// Then parse normally
```

---

### 2. Command Buffer Overflow (Possible)
**Symptom:** Rapid teleop commands corrupt Arduino serial buffer

**Evidence:**
- Teleop sends commands at ~10Hz
- Arduino processes commands slower
- Buffer fills up, M4 parsing corrupted

**Fix:** Increase Arduino serial buffer or add rate limiting in Python:
```python
# In cmd_vel_callback, add throttle:
current_time = time.time()
if hasattr(self, '_last_cmd_time'):
    if current_time - self._last_cmd_time < 0.1:  # 10Hz max
        return
self._last_cmd_time = current_time
```

---

### 3. PWM Dead Zone (Unlikely)
**Symptom:** M4 requires higher PWM to overcome friction

**Evidence:**
- Manual `m 0:0:0:100` works (100 PWM is enough)
- Teleop might send lower values during slow turns

**Fix:** Already added dead zone compensation (MIN_PWM = 40)

---

### 4. Motor Driver Channel Issue (Unlikely)
**Symptom:** Driver channel 4 intermittent failure

**Evidence:**
- Manual commands work consistently
- Teleop commands fail consistently

**Fix:** Swap wires to test, or replace driver

---

## Next Steps

1. **Run Test 1-4 above** and paste logs
2. **Check Arduino code** for M4 parsing
3. **Try manual serial commands** while ROS node is running
4. **Swap M3/M4 wires** to isolate hardware vs. software

## Expected Outcome

**If logs show:**
- `PWM: M4(RR)=255` but motor doesn't spin → **Arduino parsing bug**
- `PWM: M4(RR)=0` when it should be 255 → **Python calculation bug** (unlikely)
- Motor spins with manual command but not ROS → **Timing/buffer issue**

---

## Files Modified for Better Debugging

### ros_arduino_bridge.py
- Added buffer clearing in `send_command()`
- Added detailed PWM logging in `cmd_vel_callback()`
- Added command echo logging for motor commands

### Next: Arduino Code Review
- Need to see Arduino parsing code to identify M4-specific bugs
- Likely issue: `substring(colon3+1)` captures trailing `\r` or `\n`
