# Cube Delivery Mission - Visual Workflow

## 🎬 Mission Flow Diagram

```
START (0, 0)
    |
    | [Navigate]
    ↓
POINT 1 (2.1, 0.0) ← Pickup Location
    |
    | [Stop Robot - 1 second]
    ↓
[Color Sensor Reading]
    |
    ├─→ Red Cube Detected? → Store 'red' in blackboard
    └─→ Blue Cube Detected? → Store 'blue' in blackboard
    |
    | [Navigate while monitoring camera]
    ↓
POINT 2 (x, y) ← Delivery Location
    |
    | [Camera actively scanning]
    | [Parallel execution:]
    | ├─ Moving toward Point 2
    | └─ Monitoring for target color
    |
    ↓
[Camera detects matching color!]
    |
    | [Stop Robot - 1 second]
    ↓
[Wait for Offload]
    |
    | [Monitor color sensor]
    | [Wait until color clears]
    ↓
[Color Cleared - Cube Offloaded!]
    |
    | [Turn 180°]
    ↓
[Facing opposite direction]
    |
    | [Reverse 4cm]
    ↓
[4cm back from delivery point]
    |
    | [Activate Stepper Motor]
    ↓
[Stepper runs for 5 seconds]
    |
    ↓
MISSION COMPLETE! ✅
```

---

## 🔄 Behavior Tree Structure

```
Sequence (memory=True)
│
├─ 1️⃣ MoveToPosition("Point1", 2.1, 0.0)
│    Input:  None
│    Output: Robot at (2.1, 0.0)
│    Status: SUCCESS when within 0.2m tolerance
│
├─ 2️⃣ StopRobot(1.0s)
│    Input:  None
│    Output: Twist(0, 0) published to /cmd_vel
│    Status: SUCCESS after 1 second
│
├─ 3️⃣ ReadColorSensor()
│    Input:  /color_sensor/rgb (ColorRGBA)
│    Output: blackboard['detected_color'] = 'red'|'blue'|'unknown'
│    Status: SUCCESS when color classified
│
├─ 4️⃣ MoveAndMonitorCamera("Point2", x, y)
│    │
│    ├─ MoveToPosition (runs continuously)
│    │   Input:  Target (x, y)
│    │   Output: Navigation commands
│    │
│    └─ MonitorCameraForColor (checks in parallel)
│        Input:  /color_detection/detected (String)
│                blackboard['detected_color'] (target)
│        Output: SUCCESS when target color seen
│        Result: Stops MoveToPosition early
│    
│    Status: SUCCESS when camera detects color OR arrival at Point2
│
├─ 5️⃣ StopRobot(1.0s)
│    Input:  None
│    Output: Twist(0, 0) published to /cmd_vel
│    Status: SUCCESS after 1 second
│
├─ 6️⃣ WaitForColorSensorClear(timeout=15s)
│    Input:  /color_sensor/rgb (ColorRGBA)
│            blackboard['detected_color'] (to monitor)
│    Output: None
│    Status: SUCCESS when target color no longer detected
│            FAILURE on timeout
│
├─ 7️⃣ Turn180Degrees(0.5 rad/s)
│    Input:  None
│    Output: Twist(0, 0.5) published to /cmd_vel
│    Duration: π / 0.5 ≈ 6.3 seconds
│    Status: SUCCESS after turn complete
│
├─ 8️⃣ ReverseDistance(4cm, 0.1 m/s)
│    Input:  None
│    Output: Twist(-0.1, 0) published to /cmd_vel
│    Duration: 0.04m / 0.1 m/s = 0.4 seconds
│    Status: SUCCESS after reverse complete
│
└─ 9️⃣ ActivateStepperMotor(-25 RPM, 400mm, wait=5s)
     Input:  None
     Output: "-25:400:0" published to /stepper/command
     Status: SUCCESS after 5 second wait
```

---

## 📊 Data Flow

```
Sensors → Behavior Tree → Actuators

INPUT TOPICS:
  /color_sensor/rgb ──────────────┐
                                  │
  /color_detection/detected ──────┤
                                  │
  /amcl_pose ─────────────────────┤──→ [Behavior Tree]
                                  │
  /navigate_to_pose/_action/result│
                                  │
  /stepper/active ────────────────┘

BLACKBOARD (shared state):
  detected_color: 'red' | 'blue' | 'unknown'

OUTPUT TOPICS:
  /cmd_vel ←──────────────────────┐
                                  │
  /color_sensor/led ←─────────────┤
                                  │
  /stepper/command ←──────────────┤──── [Behavior Tree]
                                  │
  /navigate_to_pose/_action/goal ←┘
```

---

## 🎭 State Machine View

```
States:
  [IDLE] → [NAVIGATING_P1] → [READING_COLOR] → [NAVIGATING_P2] → 
  [CAMERA_MONITORING] → [WAITING_OFFLOAD] → [TURNING] → 
  [REVERSING] → [STEPPER_ACTIVE] → [COMPLETE]

Transitions:
  IDLE → NAVIGATING_P1
    Trigger: Mission start
    
  NAVIGATING_P1 → READING_COLOR
    Trigger: Robot within 0.2m of Point 1
    
  READING_COLOR → NAVIGATING_P2
    Trigger: Color classified and stored
    
  NAVIGATING_P2 → CAMERA_MONITORING
    Trigger: Immediate (parallel execution)
    
  CAMERA_MONITORING → WAITING_OFFLOAD
    Trigger: Camera detects matching color
    
  WAITING_OFFLOAD → TURNING
    Trigger: Color sensor clears (cube removed)
    
  TURNING → REVERSING
    Trigger: 180° rotation complete
    
  REVERSING → STEPPER_ACTIVE
    Trigger: 4cm reverse complete
    
  STEPPER_ACTIVE → COMPLETE
    Trigger: 5 seconds elapsed
```

---

## 🧩 Component Integration

```
┌─────────────────────────────────────────────────────────────┐
│                     BEHAVIOR TREE                            │
│  (cube_delivery_mission.py + sensor_behaviors.py)           │
└─────────────────────────────────────────────────────────────┘
    ↕                    ↕                    ↕
┌─────────────┐  ┌─────────────────┐  ┌─────────────────────┐
│   Nav2      │  │  ros_arduino    │  │  rpi_camera_package │
│ (navigation)│  │     _bridge     │  │  (color detection)  │
└─────────────┘  └─────────────────┘  └─────────────────────┘
    ↕                    ↕                    ↕
┌─────────────┐  ┌─────────────────┐  ┌─────────────────────┐
│  Costmaps   │  │    Arduino      │  │    Raspberry Pi     │
│  Localize   │  │   (Sensors/     │  │      Camera         │
│  SLAM       │  │    Motors)      │  │    (V4L2/libcamera) │
└─────────────┘  └─────────────────┘  └─────────────────────┘
```

---

## ⏱️ Timing Analysis

```
Mission Phase               Estimated Duration    Notes
────────────────────────────────────────────────────────────
1. Navigate to Point 1      10-30 seconds        Depends on distance
2. Stop at Point 1          1 second             Stabilization
3. Read color sensor        2 seconds            LED on, read, classify
4. Navigate to Point 2      10-30 seconds        Depends on distance
   + Camera monitoring      (parallel)           Real-time detection
5. Stop at Point 2          1 second             When camera detects
6. Wait for offload         0-15 seconds         Manual or automatic
7. Turn 180 degrees         6.3 seconds          At 0.5 rad/s
8. Reverse 4cm              0.4 seconds          At 0.1 m/s
9. Activate stepper         5 seconds            Motor operation

────────────────────────────────────────────────────────────
TOTAL (typical):            ~45-90 seconds       ~1-1.5 minutes
TOTAL (worst case):         ~70-120 seconds      ~1-2 minutes
```

---

## 🚦 Decision Points

```
Decision Tree:

Start Mission
│
├─ Can reach Point 1?
│  ├─ YES → Continue
│  └─ NO → FAILURE (navigation blocked)
│
├─ Color sensor working?
│  ├─ YES → Read color
│  └─ NO → FAILURE (sensor error)
│
├─ Color detected?
│  ├─ RED → Store 'red', continue
│  ├─ BLUE → Store 'blue', continue
│  └─ UNKNOWN → Store 'unknown', continue (may fail at camera match)
│
├─ Can navigate toward Point 2?
│  ├─ YES → Start moving
│  └─ NO → FAILURE (navigation blocked)
│
├─ Camera detects matching color?
│  ├─ BEFORE Point 2 → Stop early, continue ✓
│  ├─ AT Point 2 → Continue to wait ⚠️
│  └─ NEVER → Keep waiting (running state)
│
├─ Color sensor clears?
│  ├─ WITHIN 15s → Continue ✓
│  └─ TIMEOUT → FAILURE (cube not offloaded)
│
├─ Turn successful?
│  ├─ YES → Continue
│  └─ NO → FAILURE (should not happen)
│
├─ Reverse successful?
│  ├─ YES → Continue
│  └─ NO → FAILURE (should not happen)
│
└─ Stepper activated?
   ├─ YES → SUCCESS ✓
   └─ NO → FAILURE (stepper error)
```

---

## 🔌 Hardware Connections

```
Arduino Mega ───┬─── TCS34725 (I2C: SDA, SCL)
                │    └─> Color sensor for cube detection
                │
                ├─── Motors (M1-M4: PWM + Direction)
                │    └─> 4WD drive system
                │
                ├─── Encoders (4x, Interrupt pins)
                │    └─> Odometry feedback
                │
                ├─── Stepper Motor (Step, Dir, Enable)
                │    └─> Offload mechanism
                │
                ├─── Servo 0 (PWM pin)
                │    └─> Camera tilt servo
                │
                └─── Servo 1 (PWM pin)
                     └─> Tipper servo (optional)

Raspberry Pi ───┬─── Camera Module (CSI or USB)
                │    └─> Color detection at Point 2
                │
                └─── USB → Arduino (Serial commands)
```

---

## 📈 Success Metrics

| Metric                  | Target      | Measurement Method                |
|-------------------------|-------------|-----------------------------------|
| Navigation accuracy     | ±20cm       | AMCL pose vs target              |
| Color detection rate    | >95%        | Sensor reading consistency       |
| Camera detection delay  | <2 seconds  | Time from color visible → stop   |
| Turn accuracy           | ±10°        | IMU or encoder-based odometry    |
| Reverse accuracy        | ±1cm        | Encoder-based distance           |
| Mission success rate    | >90%        | Complete runs / total attempts   |
| Average mission time    | 45-90s      | Start → completion timer         |

---

## 🛡️ Safety & Recovery

```
Safety Checks:
  ✓ Obstacle avoidance (Nav2 costmaps)
  ✓ Timeout on all waiting states
  ✓ Robot stops on behavior failure
  ✓ Emergency stop via /cmd_vel(0,0)

Recovery Behaviors:
  ⚠️ Navigation blocked → Nav2 recovery (rotate, back up)
  ⚠️ Color sensor timeout → Mark as 'unknown', continue
  ⚠️ Camera never detects → Reach Point 2, wait indefinitely
  ⚠️ Offload timeout → FAILURE, mission aborts
  
Manual Interventions:
  🔧 Clear obstacles from path
  🔧 Manually position cube if color not detected
  🔧 Manually offload cube if stuck
  🔧 Press Ctrl+C to abort mission
```

---

This visual guide provides a comprehensive overview of the cube delivery mission architecture and workflow! 🎯
