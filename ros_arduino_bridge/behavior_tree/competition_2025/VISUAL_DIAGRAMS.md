# 🎨 VISUAL SYSTEM ARCHITECTURE

## 📐 System Overview Diagram

```
╔═══════════════════════════════════════════════════════════════════╗
║                         COMPETITION ROBOT                         ║
║                      Split Architecture                           ║
╚═══════════════════════════════════════════════════════════════════╝

┌───────────────────────────────────────────────────────────────────┐
│                    🤖 RASPBERRY PI (Robot)                        │
│                      Hardware Layer                                │
├───────────────────────────────────────────────────────────────────┤
│                                                                    │
│  ┌─────────────┐  ┌──────────┐  ┌────────┐  ┌──────────────┐   │
│  │  Arduino    │  │  LiDAR   │  │ Camera │  │ TF Publisher │   │
│  │   Bridge    │  │  Driver  │  │  Node  │  │    (URDF)    │   │
│  └──────┬──────┘  └────┬─────┘  └────┬───┘  └──────┬───────┘   │
│         │              │              │              │            │
│         ├─ /odom       ├─ /scan       ├─ /image     ├─ /tf      │
│         ├─ /imu/data   │              │              │            │
│         ├─ /color_sensor/rgb          │              │            │
│         │              │              │              │            │
│         └──────────────┴──────────────┴──────────────┘            │
│                         │ ROS2 Network                             │
│                         ↓ (Topics published)                       │
└───────────────────────────────────────────────────────────────────┘
                            │
                            │ WiFi/Ethernet
                            │ (Automatic DDS discovery)
                            ↓
┌───────────────────────────────────────────────────────────────────┐
│                    💻 LAPTOP (Base Station)                       │
│                    Processing Layer                                │
├───────────────────────────────────────────────────────────────────┤
│                                                                    │
│  ╔════════════════════════════════════════════════════════════╗  │
│  ║              🌳 BEHAVIOR TREE (Main Brain)                 ║  │
│  ╠════════════════════════════════════════════════════════════╣  │
│  ║                                                            ║  │
│  ║  ┌──────────────────────────────────────────────────┐     ║  │
│  ║  │  Phase 1: Disease Detection                      │     ║  │
│  ║  │  ┌────────────┐  ┌───────────┐  ┌────────────┐  │     ║  │
│  ║  │  │ Navigate   │→ │ Run ML    │→ │ Return     │  │     ║  │
│  ║  │  │ to Plant   │  │ Inference │  │ Home       │  │     ║  │
│  ║  │  └────────────┘  └───────────┘  └────────────┘  │     ║  │
│  ║  └──────────────────────────────────────────────────┘     ║  │
│  ║                                                            ║  │
│  ║  ┌──────────────────────────────────────────────────┐     ║  │
│  ║  │  Phase 2: Cargo Loading                          │     ║  │
│  ║  │  ┌────────────┐  ┌───────────┐  ┌────────────┐  │     ║  │
│  ║  │  │ Navigate   │→ │ Reverse   │→ │ Read Color │  │     ║  │
│  ║  │  │ to Bay     │  │ Into Bay  │  │ Sensor     │  │     ║  │
│  ║  │  └────────────┘  └───────────┘  └────────────┘  │     ║  │
│  ║  └──────────────────────────────────────────────────┘     ║  │
│  ║                                                            ║  │
│  ║  ┌──────────────────────────────────────────────────┐     ║  │
│  ║  │  Phase 3: Maze Navigation                        │     ║  │
│  ║  │  ┌────────────┐  ┌───────────┐                   │     ║  │
│  ║  │  │ Navigate   │∥ │ Monitor   │                   │     ║  │
│  ║  │  │ to Delivery│∥ │ Camera    │ (Parallel)        │     ║  │
│  ║  │  └────────────┘  └───────────┘                   │     ║  │
│  ║  └──────────────────────────────────────────────────┘     ║  │
│  ║                                                            ║  │
│  ║  ┌──────────────────────────────────────────────────┐     ║  │
│  ║  │  Phase 4: Cargo Delivery                         │     ║  │
│  ║  │  ┌────────────┐  ┌───────────┐  ┌────────────┐  │     ║  │
│  ║  │  │ Reverse    │→ │ Activate  │→ │ Reset      │  │     ║  │
│  ║  │  │ Into Bay   │  │ Conveyor  │  │ Robot      │  │     ║  │
│  ║  │  └────────────┘  └───────────┘  └────────────┘  │     ║  │
│  ║  └──────────────────────────────────────────────────┘     ║  │
│  ║                                                            ║  │
│  ╚════════════════════════════════════════════════════════════╝  │
│                            ↓                                       │
│                            ↓ (Uses these subsystems)               │
│                            ↓                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐    │
│  │   Nav2       │  │  ML Models   │  │  Vision Processing  │    │
│  │              │  │              │  │                     │    │
│  │ • AMCL       │  │ • Disease    │  │ • Color Detection  │    │
│  │ • Planner    │  │   Detection  │  │   (OpenCV HSV)     │    │
│  │ • Controller │  │   (TF/PyT)   │  │ • HSV Filtering    │    │
│  │ • Costmap    │  │              │  │ • Contour Finding  │    │
│  │              │  │              │  │                     │    │
│  └──────┬───────┘  └──────┬───────┘  └─────────┬───────────┘    │
│         │                 │                     │                 │
│         └─────────────────┴─────────────────────┘                 │
│                           │                                        │
│                           ↓ (Publishes commands)                   │
│                                                                    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Published Topics:                                          │  │
│  │  • /cmd_vel         → Velocity commands                    │  │
│  │  • /camera_servo    → Camera pan                           │  │
│  │  • /tipper_servo    → Robot tilt                           │  │
│  │  • /conveyor        → Belt control                         │  │
│  │  • /navigate_to_pose → Nav2 goals                          │  │
│  └────────────────────────────────────────────────────────────┘  │
│                           │                                        │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Subscribed Topics:                                         │  │
│  │  • /odom            ← Odometry                             │  │
│  │  • /scan            ← LiDAR data                           │  │
│  │  • /image_raw       ← Camera feed                          │  │
│  │  • /color_sensor    ← RGB values                           │  │
│  │  • /amcl_pose       ← Robot position                       │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  RViz Visualization:                                        │  │
│  │  • Map display                                              │  │
│  │  • Robot model                                              │  │
│  │  • Planned paths                                            │  │
│  │  • Costmaps                                                 │  │
│  │  • Sensor data                                              │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                    │
└───────────────────────────────────────────────────────────────────┘
```

## 🔄 Autonomy Loop

```
START
  │
  ├─→ [Behavior Tree Tick] (every 0.5s)
  │       │
  │       ├─→ Check current behavior status
  │       │
  │       ├─→ Is it RUNNING?
  │       │   ├─ YES → Read sensor topics
  │       │   │         ├─ /amcl_pose (where am I?)
  │       │   │         ├─ /scan (any obstacles?)
  │       │   │         ├─ /color_sensor/rgb (what color?)
  │       │   │         └─ /camera/color_detection (zone color?)
  │       │   │
  │       │   └─ Update behavior state
  │       │       ├─ Distance to goal?
  │       │       ├─ Color detected?
  │       │       ├─ ML inference done?
  │       │       └─ Return: RUNNING | SUCCESS | FAILURE
  │       │
  │       ├─→ Is it SUCCESS?
  │       │   └─ YES → Move to next behavior
  │       │
  │       └─→ Is it FAILURE?
  │           └─ YES → Try fallback or report error
  │
  ├─→ [Nav2 Loop] (continuous)
  │       │
  │       ├─→ Receive goal from behavior tree
  │       ├─→ Plan path (avoid obstacles)
  │       ├─→ Execute path (DWB controller)
  │       ├─→ Publish /cmd_vel → Pi
  │       └─→ Pi moves robot → publishes /odom
  │               │
  │               └─→ Nav2 reads /odom, adjusts path
  │
  └─→ [Sensor Loop] (continuous)
          │
          ├─→ Pi reads hardware
          ├─→ Pi publishes topics
          ├─→ Laptop processes topics
          │   ├─ AMCL: updates robot pose
          │   ├─ Costmap: updates obstacles
          │   ├─ ML: runs inference
          │   └─ Color: detects colors
          │
          └─→ Behavior tree reads processed data
              └─→ Makes decisions
                  └─→ Loop continues!

NO HUMAN INTERVENTION! 🎉
```

## 🎯 Command Flow Example: "Navigate to Loading Bay"

```
1. Behavior Tree (Laptop)
   ↓
   behavior = MoveToPosition("ToLoadingBay", 0.3, 0.8)
   behavior.update()
   ↓

2. Check AMCL pose (Laptop)
   ↓
   current_pose = read /amcl_pose topic
   current_x = 0.0, current_y = 0.0
   ↓

3. Send goal to Nav2 (Laptop)
   ↓
   goal = NavigateToPose.Goal()
   goal.pose.position.x = 0.3
   goal.pose.position.y = 0.8
   send_goal_async(goal)
   ↓

4. Nav2 plans path (Laptop)
   ↓
   global_planner.plan(current_pose → goal_pose)
   path = [(0.0,0.0), (0.1,0.2), (0.2,0.5), (0.3,0.8)]
   ↓

5. Nav2 executes path (Laptop)
   ↓
   controller.follow_path(path)
   ↓
   velocity = calculate_velocity(current_pose, next_waypoint)
   publish /cmd_vel {linear: 0.3, angular: 0.5}
   ↓

6. Arduino receives command (Pi)
   ↓
   receive /cmd_vel from network
   left_speed = (linear - angular * base_width/2)
   right_speed = (linear + angular * base_width/2)
   write to Arduino: "m 150 150"  # motor commands
   ↓

7. Robot moves (Pi hardware)
   ↓
   motors spin at commanded speeds
   encoders count ticks
   IMU measures rotation
   ↓

8. Arduino publishes odometry (Pi)
   ↓
   read encoder ticks: left=373, right=380
   calculate: dx, dy, dtheta
   publish /odom {x=0.05, y=0.02, theta=0.01}
   ↓

9. AMCL updates pose (Laptop)
   ↓
   receive /odom from network
   fuse with /scan data
   update particle filter
   publish /amcl_pose {x=0.05, y=0.02, ...}
   ↓

10. Behavior tree checks progress (Laptop)
    ↓
    distance = sqrt((0.3-0.05)² + (0.8-0.02)²) = 0.79m
    if distance < tolerance (0.2m):
        return SUCCESS
    else:
        return RUNNING
    ↓

11. Loop back to step 4
    (continues until distance < tolerance)
    ↓

12. Behavior returns SUCCESS
    ↓
    Move to next behavior in tree!
```

## 📊 Topic Bandwidth Estimate

```
PUBLISHED BY PI:
  /odom                    : ~50 Hz, ~1 KB/msg  = 50 KB/s
  /scan                    : ~10 Hz, ~5 KB/msg  = 50 KB/s
  /image_raw/compressed    : ~30 Hz, ~20 KB/msg = 600 KB/s
  /color_sensor/rgb        : ~5 Hz, ~0.1 KB/msg = 0.5 KB/s
  /tf                      : ~50 Hz, ~1 KB/msg  = 50 KB/s
                                         TOTAL  ≈ 750 KB/s

PUBLISHED BY LAPTOP:
  /cmd_vel                 : ~20 Hz, ~0.2 KB/msg = 4 KB/s
  /camera_servo/command    : ~1 Hz, ~0.1 KB/msg  = 0.1 KB/s
  /conveyor/command        : ~1 Hz, ~0.1 KB/msg  = 0.1 KB/s
                                         TOTAL  ≈ 4 KB/s

Network Load: ~754 KB/s ≈ 6 Mbps (WiFi handles easily!)
```

## 🏁 Timeline: Full Mission Execution

```
Time    Pi Activity                Laptop Activity
────    ───────────────            ───────────────
0:00    Publish sensors            Load map
0:01    Publish sensors            Start AMCL
0:02    Publish sensors            Wait for initial pose
0:05    Publish sensors            Initial pose set
                                   Behavior tree starts!

0:06    Receive /cmd_vel           Phase 1: Disease Detection
0:07    Move robot                 Navigate to plant
0:08    Publish /odom              Monitor progress
...
0:20    Capture image              ML inference running
0:21    Publish image              Result: "Healthy"
...
0:35    Receive /cmd_vel           Phase 2: Loading
0:36    Move backward              Reverse into bay
0:38    Read RGB sensor            Waiting for color
0:39    Publish RGB: "red"         Color detected!
...
0:50    Receive /cmd_vel           Phase 3: Maze
0:51    Move to red zone           Navigate + monitor camera
...
1:30    Capture image              Camera sees red marker
1:31    Publish image              Verify zone match!
...
1:45    Move backward              Phase 4: Delivery
1:47    Receive conveyor cmd       Reverse into bay
1:48    Activate conveyor          Activate conveyor
1:52    Cargo offloaded!           SUCCESS!
                                   MISSION COMPLETE! 🎉

Total Time: ~2 minutes
```

---

**All diagrams show: Pi handles hardware, Laptop makes decisions!**
