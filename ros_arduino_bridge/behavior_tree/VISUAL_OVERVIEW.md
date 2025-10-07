# Visual System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                    COMPETITION MISSION SYSTEM                        │
│                      (Autonomous Robot)                              │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  HARDWARE LAYER                                                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │
│  │ Arduino  │  │  LiDAR   │  │ Pi Cam   │  │  Servos  │           │
│  │  + RGB   │  │ (SLAM)   │  │ (Color)  │  │Conveyor  │           │
│  │  Sensor  │  │          │  │          │  │          │           │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │
│       │             │             │             │                    │
└───────┼─────────────┼─────────────┼─────────────┼────────────────────┘
        │             │             │             │
        ▼             ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  ROS2 COMMUNICATION LAYER                                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  /color_sensor/rgb        /scan              /camera/image_raw      │
│  /color_sensor/led        /map               /camera/color_detection│
│  /camera_servo/command    /amcl_pose         /inference_result      │
│  /tipper_servo/command    /cmd_vel           /stepper/command       │
│  /navigate_to_pose (Nav2 Action)                                    │
│                                                                       │
└───────────────────────────────────────────────┬───────────────────────┘
                                                │
                                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│  BEHAVIOR NODE LAYER (Your New Modular Nodes!)                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌────────────────────┐  ┌────────────────────┐                    │
│  │ Color Sensor Nodes │  │  Camera Nodes      │                    │
│  ├────────────────────┤  ├────────────────────┤                    │
│  │ ReadColorSensor    │  │ MonitorCameraFor   │                    │
│  │ WaitForClear       │  │   Color            │                    │
│  └────────────────────┘  │ VerifyColorMatch   │                    │
│                           └────────────────────┘                    │
│  ┌────────────────────┐  ┌────────────────────┐                    │
│  │ Navigation Nodes   │  │ Motor Control      │                    │
│  ├────────────────────┤  ├────────────────────┤                    │
│  │ MoveToPosition     │  │ ActivateCamera     │                    │
│  │ ReverseDistance    │  │   Servo            │                    │
│  │ Turn180Degrees     │  │ ActivateTipper     │                    │
│  │ StopRobot          │  │   Servo            │                    │
│  └────────────────────┘  │ ActivateConveyor   │                    │
│                           └────────────────────┘                    │
│  ┌────────────────────┐                                             │
│  │ Disease Detection  │                                             │
│  ├────────────────────┤                                             │
│  │ WaitForDisease     │                                             │
│  │   Detection        │                                             │
│  │ LogDiseaseResult   │                                             │
│  └────────────────────┘                                             │
│                                                                       │
└───────────────────────────────────────────────┬───────────────────────┘
                                                │
                                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│  BEHAVIOR TREE LAYER (Mission Orchestration)                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  CompetitionMission (Root Sequence)                                  │
│  │                                                                    │
│  ├─ Phase 1: Disease Detection (Selector - can skip)                │
│  │  └─ Navigate → Adjust Camera → Detect → Log → Return            │
│  │                                                                    │
│  ├─ Phase 2: Cargo Loading (Sequence)                               │
│  │  └─ Navigate → Reverse In → Read Color → Exit                   │
│  │                                                                    │
│  ├─ Phase 3: Maze Navigation (Sequence)                             │
│  │  └─ Enter Maze → Navigate to Delivery (Selector)                │
│  │     ├─ Red Branch → Red Delivery Zone                           │
│  │     ├─ Blue Branch → Blue Delivery Zone                         │
│  │     ├─ Green Branch → Green Delivery Zone                       │
│  │     └─ Manual Fallback (always succeeds)                        │
│  │                                                                    │
│  └─ Phase 4: Cargo Delivery (Sequence)                              │
│     └─ Verify Zone → Turn 180° → Reverse In → Offload → Exit      │
│        └─ Offload with Retry:                                       │
│           ├─ Attempt 1: Conveyor only                              │
│           └─ Attempt 2: Conveyor + Tipper (if needed)              │
│                                                                       │
└───────────────────────────────────────────────┬───────────────────────┘
                                                │
                                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│  APPLICATION LAYER (What You Run)                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌─────────────────────┐  ┌─────────────────────┐                  │
│  │ competition_mission │  │ launch_competition  │                  │
│  │      .py            │  │       .py           │                  │
│  │                     │  │                     │                  │
│  │ Main mission file   │  │ Easy launcher with  │                  │
│  │ with all phases     │  │ prerequisite checks │                  │
│  └─────────────────────┘  └─────────────────────┘                  │
│                                                                       │
│  ┌─────────────────────┐                                            │
│  │  test_behaviors.py  │                                            │
│  │                     │                                            │
│  │  Test individual    │                                            │
│  │  components         │                                            │
│  └─────────────────────┘                                            │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════
                        MISSION FLOW DIAGRAM
═══════════════════════════════════════════════════════════════════════

START
  │
  ├─ [Optional] Disease Detection Phase
  │     │
  │     ├─→ Navigate to plant display
  │     ├─→ Adjust camera servo (45°)
  │     ├─→ Wait for ML classification
  │     │    └─→ Timeout (15s) → Manual observation
  │     ├─→ Log result for judges
  │     └─→ Return to start
  │
  ├─ Cargo Loading Phase
  │     │
  │     ├─→ Navigate to loading bay
  │     ├─→ Reverse into bay (0.5m)  ← REAR LOADING!
  │     ├─→ Stop & wait (5s)
  │     ├─→ Read color sensor
  │     │    ├─→ Success: Store color in blackboard
  │     │    └─→ Timeout: Store 'unknown' → Manual loading
  │     └─→ Exit bay (drive forward 0.5m)
  │
  ├─ Maze Navigation Phase
  │     │
  │     ├─→ Enter maze
  │     │
  │     └─→ Select delivery zone based on color:
  │          │
  │          ├─→ [If RED] Navigate to red zone
  │          │    └─→ Camera monitors for red during navigation
  │          │         └─→ Stop when red detected
  │          │
  │          ├─→ [If BLUE] Navigate to blue zone
  │          │    └─→ Camera monitors for blue during navigation
  │          │         └─→ Stop when blue detected
  │          │
  │          ├─→ [If GREEN] Navigate to green zone
  │          │    └─→ Camera monitors for green during navigation
  │          │         └─→ Stop when green detected
  │          │
  │          └─→ [If UNKNOWN] Navigate to default zone
  │               └─→ Manual intervention expected
  │
  │          Note: Nav2 handles obstacle avoidance automatically!
  │
  └─ Cargo Delivery Phase
        │
        ├─→ Verify at correct zone (camera check)
        │    ├─→ Success: Continue
        │    └─→ Fail: Log error but continue (manual fix)
        │
        ├─→ Turn 180° to face away from zone
        │
        ├─→ Reverse into delivery bay (0.4m)
        │
        ├─→ Offload cargo with retry:
        │    │
        │    ├─→ Attempt 1: Activate conveyor (4s, speed=200)
        │    ├─→ Check if cargo cleared (color sensor)
        │    │    ├─→ Clear: Success! → Exit
        │    │    └─→ Not clear: Continue to Attempt 2
        │    │
        │    ├─→ Attempt 2: Activate tipper servo (90°) + conveyor (5s, speed=255)
        │    ├─→ Check if cargo cleared
        │    │    ├─→ Clear: Success!
        │    │    └─→ Not clear: Assume manual intervention
        │    │
        │    └─→ Reset tipper servo (170°)
        │
        ├─→ Wait (2s)
        │
        └─→ Exit delivery bay (0.5m forward)

MISSION COMPLETE! 🎉


═══════════════════════════════════════════════════════════════════════
                      DATA FLOW DIAGRAM
═══════════════════════════════════════════════════════════════════════

Sensors → ROS Topics → Behavior Nodes → Blackboard → Mission Decisions

Example: Color-Based Delivery

  1. Color Sensor        → /color_sensor/rgb
     (Hardware)             (ROS Topic)
                               ↓
  2. ReadColorSensor     → Classify RGB
     (Behavior Node)        (red/blue/green/unknown)
                               ↓
  3. Blackboard          → Store 'detected_color'
     (Shared Memory)        (e.g., 'red')
                               ↓
  4. CheckBlackboardColor→ Compare with expected
     (Behavior Node)        (Is it 'red'?)
                               ↓
  5. Selector            → Choose delivery zone
     (Tree Logic)           (Navigate to red zone)
                               ↓
  6. MoveToPosition      → /navigate_to_pose action
     (Behavior Node)        (Nav2 navigation)
                               ↓
  7. Robot Arrives       → Camera verifies color
     (Physical Movement)    (VerifyColorMatch node)


═══════════════════════════════════════════════════════════════════════
                    FALLBACK MECHANISM DIAGRAM
═══════════════════════════════════════════════════════════════════════

Every critical operation has a backup plan:

┌─────────────────────────────────────────────────────────────────────┐
│ SENSOR READING                                                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Primary: Wait for sensor data (15s)                                │
│     │                                                                 │
│     ├─→ Success → Continue with data                                │
│     │                                                                 │
│     └─→ Timeout → Store 'unknown' → ALLOW MANUAL INTERVENTION       │
│         (Mission continues instead of failing)                       │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ NAVIGATION                                                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Primary: Nav2 path to goal                                         │
│     │                                                                 │
│     ├─→ Obstacle detected → Nav2 REPLANS AUTOMATICALLY              │
│     │   (Dynamic costmap update, new path calculated)               │
│     │                                                                 │
│     └─→ Path blocked completely → Nav2 reports failure              │
│         → Behavior tree can retry or skip                           │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ CARGO OFFLOAD                                                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Attempt 1: Conveyor only (4s)                                      │
│     │                                                                 │
│     ├─→ Check cleared (color sensor)                                │
│     │    │                                                            │
│     │    ├─→ Success → Done!                                        │
│     │    │                                                            │
│     │    └─→ Not cleared → Try Attempt 2                            │
│     │                                                                 │
│  Attempt 2: Conveyor + Tipper servo (5s)                            │
│     │                                                                 │
│     ├─→ Check cleared                                               │
│     │    │                                                            │
│     │    ├─→ Success → Done!                                        │
│     │    │                                                            │
│     │    └─→ Not cleared → ASSUME MANUAL FIX                        │
│     │        (Mission continues)                                     │
│     │                                                                 │
│     └─→ Reset tipper servo                                          │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════
                        FILE DEPENDENCIES
═══════════════════════════════════════════════════════════════════════

competition_mission.py (Main File)
│
├─ Imports from: nodes/__init__.py
│  │
│  ├─ color_sensor_behaviors.py
│  │  └─ Uses: /color_sensor/rgb, /color_sensor/led topics
│  │
│  ├─ camera_behaviors.py
│  │  └─ Uses: /camera/color_detection topic
│  │
│  ├─ disease_detection_behaviors.py
│  │  └─ Uses: /inference_result topic
│  │
│  ├─ navigation_behaviors.py
│  │  └─ Uses: /cmd_vel, /navigate_to_pose action
│  │
│  └─ motor_control_behaviors.py
│     └─ Uses: /camera_servo/command, /tipper_servo/command,
│               /stepper/command topics
│
└─ External Dependencies:
   ├─ py_trees (behavior tree library)
   ├─ py_trees_ros (ROS2 integration)
   ├─ rclpy (ROS2 Python client)
   └─ Nav2 (navigation stack)


═══════════════════════════════════════════════════════════════════════
                    TESTING WORKFLOW
═══════════════════════════════════════════════════════════════════════

Individual Component Testing (Bottom-Up)
│
├─ test_behaviors.py color_sensor
│  └─ Tests: ReadColorSensor node
│     └─ Verifies: RGB reading, color classification
│
├─ test_behaviors.py camera
│  └─ Tests: MonitorCameraForColor node
│     └─ Verifies: Camera detection, color matching
│
├─ test_behaviors.py motors
│  └─ Tests: All servo and conveyor nodes
│     └─ Verifies: Servo movements, conveyor activation
│
├─ test_behaviors.py navigation
│  └─ Tests: Movement behaviors
│     └─ Verifies: Forward, reverse, turn movements
│
└─ test_behaviors.py disease
   └─ Tests: Disease detection workflow
      └─ Verifies: ML model integration

Integration Testing (Top-Down)
│
├─ launch_competition.py --dry-run
│  └─ Validates: Tree structure, no execution
│
├─ Test Phase by Phase
│  ├─ Phase 1 only (disease detection)
│  ├─ Phase 2 only (cargo loading)
│  ├─ Phase 3 only (maze navigation)
│  └─ Phase 4 only (cargo delivery)
│
└─ Full Mission Rehearsal
   └─ launch_competition.py
      └─ Monitors: All phases, timing, performance


═══════════════════════════════════════════════════════════════════════

Created: October 7, 2025
Status: READY FOR COMPETITION! 🏆
```
