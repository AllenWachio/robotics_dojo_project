# 🏆 COMPETITION 2025 - COMPLETE PACKAGE

## 📖 Documentation Index

### **START HERE**

1. **[QUICK_START.md](QUICK_START.md)** ⚡

   - Competition day commands
   - 3-step launch process
   - Emergency commands
   - < 2 page quick reference

2. **[DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)** 📘
   - Full split architecture explanation
   - How autonomy works
   - Troubleshooting guide
   - Resource usage info

### **Technical Reference**

3. **[PYTREES_COMPARISON.md](PYTREES_COMPARISON.md)** 🔍

   - Reference code vs our implementation
   - What we learned and adopted
   - How waypoints were calculated
   - Key improvements made

4. **[ARCHITECTURE_EXPLAINED.md](ARCHITECTURE_EXPLAINED.md)** 🎯

   - Pi vs Laptop responsibilities
   - Data flow examples
   - Why this approach works
   - Before/after comparison

5. **[VISUAL_DIAGRAMS.md](VISUAL_DIAGRAMS.md)** 🎨

   - System architecture diagram
   - Autonomy loop flowchart
   - Command flow example
   - Timeline visualization

6. **[README.md](README.md)** 📚
   - Code organization
   - Behavior node details
   - Waypoint configuration
   - Mission phases

---

## 🚀 Quick Launch Commands

### **Competition Day (3 Steps):**

```bash
# Step 1: On Raspberry Pi
./launch/01_pi_hardware.sh

# Step 2: On Laptop
./launch/02_laptop_processing.sh

# Step 3: On Laptop (new terminal)
./launch/03_run_mission.sh
```

---

## 📁 File Structure

```
competition_2025/
│
├── 📄 INDEX.md (this file)
│
├── 📚 Documentation
│   ├── QUICK_START.md              ⚡ Start here for competition!
│   ├── UPDATE_SUMMARY.md           📝 Latest changes (waypoints updated!)
│   ├── FIELD_MAP.md                🗺️ Field layout with coordinates
│   ├── PYTREES_COMPARISON.md       🔍 Reference code analysis
│   ├── USING_EXISTING_FILES.md     🔗 Shows which of YOUR files are used
│   ├── DEPLOYMENT_GUIDE.md         📘 Full deployment guide
│   ├── ARCHITECTURE_EXPLAINED.md   🎯 Split architecture explained
│   ├── VISUAL_DIAGRAMS.md          🎨 System diagrams
│   └── README.md                   📖 Technical reference
│
├── 🚀 Launch Scripts
│   ├── 01_pi_hardware.sh           🤖 Run on Pi
│   ├── 02_laptop_processing.sh     💻 Run on laptop
│   └── 03_run_mission.sh           🌳 Main mission
│
├── 🧠 Code
│   ├── competition_mission.py      Main behavior tree
│   └── behaviors/                   Behavior node modules
│       ├── navigation.py           Nav2 integration
│       ├── color_sensor.py         RGB sensor
│       ├── camera.py               Camera processing
│       ├── disease_detection.py    ML model
│       └── actuators.py            Motors/servos
│
└── 🧪 Testing (to be added)
    └── testing/
```

---

## 🎓 Learning Path

### **If you're new to the system:**

1. Read **QUICK_START.md** (2 min)
2. Skim **ARCHITECTURE_EXPLAINED.md** (5 min)
3. Look at **VISUAL_DIAGRAMS.md** (understand data flow)
4. Try launching (follow QUICK_START)

### **If you're debugging:**

1. Check **DEPLOYMENT_GUIDE.md** → Troubleshooting section
2. Review **VISUAL_DIAGRAMS.md** → Command flow
3. Check topic connections: `ros2 topic list`
4. Monitor specific topics: `ros2 topic echo <topic>`

### **If you're modifying code:**

1. Read **README.md** → Behavior nodes section
2. Check `behaviors/` → Find relevant behavior
3. Edit behavior or add new one
4. Update waypoints in `competition_mission.py`
5. Test with `03_run_mission.sh`

---

## ❓ Common Questions

### "Is it fully autonomous?"

**YES!** Once you start `03_run_mission.sh`, the robot:

- Makes all decisions (behavior tree)
- Navigates autonomously (Nav2)
- Detects colors/diseases (sensors + ML)
- Completes mission without human input

See: **DEPLOYMENT_GUIDE.md** → "Autonomy Explained"

### "What runs on Pi vs Laptop?"

- **Pi**: Hardware only (sensors, motors)
- **Laptop**: Processing (Nav2, ML, behavior tree)

See: **ARCHITECTURE_EXPLAINED.md** → "Split Architecture"

### "How do I change waypoints?"

Edit `competition_mission.py`:

```python
WAYPOINTS = {
    'loading_bay': (0.3, 0.8),  # ← Change these coordinates
    'red_delivery': (1.5, -0.8),
    ...
}
```

See: **README.md** → "Waypoints"

### "How do I add a new behavior?"

1. Create file in `behaviors/`
2. Inherit from `py_trees.behaviour.Behaviour`
3. Implement `update()` method
4. Add to `behaviors/__init__.py`
5. Use in `competition_mission.py`

See: **README.md** → "Behavior Nodes"

### "What if navigation fails?"

Check in order:

1. Is Pi hardware running? (`ros2 topic list | grep odom`)
2. Is Nav2 running? (`ros2 node list | grep amcl`)
3. Is initial pose set? (RViz particles converged?)
4. Are waypoints valid? (Not inside walls?)

See: **DEPLOYMENT_GUIDE.md** → "Troubleshooting"

---

## 🛠️ Customization Guide

### **Change Mission Timing:**

```python
# In competition_mission.py
detect = WaitForDiseaseDetection("DetectDisease", timeout=15.0)  # ← Change timeout
```

### **Adjust Navigation Tolerance:**

```python
# In competition_mission.py
move = MoveToPosition("ToGoal", x, y, tolerance=0.2)  # ← Change tolerance
```

### **Change Delivery Behavior:**

```python
# In competition_mission.py, create_cargo_delivery_phase()
offload = ActivateConveyorBelt("Offload", duration=3.0, pwm=200)  # ← Adjust duration/speed
```

### **Disable Disease Detection:**

```python
# In competition_mission.py, create_root()
# Comment out disease phase:
# disease_phase = create_disease_detection_phase()

root.add_children([
    # disease_phase,  # ← Commented out
    loading_phase,
    maze_phase,
    delivery_phase,
])
```

---

## 🔧 Development Workflow

### **Typical workflow:**

```bash
# 1. Code on laptop
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree/competition_2025
nano competition_mission.py

# 2. Build workspace
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge
source install/setup.bash

# 3. Test with real robot
./launch/01_pi_hardware.sh     # On Pi
./launch/02_laptop_processing.sh  # On laptop
./launch/03_run_mission.sh     # On laptop

# 4. Debug if needed
ros2 topic echo /amcl_pose
ros2 topic echo /color_sensor/rgb
ros2 node list
```

### **Git workflow:**

```bash
# Stage changes
git add competition_2025/

# Commit
git commit -m "Updated waypoints for competition field"

# Push
git push origin making_pytrees_work_well
```

---

## 📊 Pre-Flight Checklist

### **Before Competition Day:**

- [ ] Test full mission on practice field
- [ ] Verify all waypoints reach correctly
- [ ] Calibrate color sensor thresholds
- [ ] Test ML disease detection
- [ ] Practice 3-step launch procedure
- [ ] Backup code to USB drive
- [ ] Print QUICK_START.md cheat sheet

### **Competition Morning:**

- [ ] Charge robot battery
- [ ] Charge laptop
- [ ] Test Pi SSH connection
- [ ] Verify map file exists
- [ ] Test Arduino connection
- [ ] Test LiDAR connection
- [ ] Test camera
- [ ] Quick test run (without obstacles)

### **5 Minutes Before Run:**

- [ ] Place robot at start position
- [ ] Launch `01_pi_hardware.sh` on Pi
- [ ] Launch `02_laptop_processing.sh` on laptop
- [ ] Set initial pose in RViz
- [ ] Verify Nav2 ready (`ros2 node list`)
- [ ] Deep breath! 😊

### **Competition Run:**

- [ ] Launch `03_run_mission.sh`
- [ ] Watch RViz (don't touch keyboard!)
- [ ] Monitor terminal output
- [ ] Time the run (target: 2.5-3 min)
- [ ] Celebrate success! 🎉

---

## 📞 Support Resources

### **Documentation:**

- **QUICK_START.md** - Fast reference
- **DEPLOYMENT_GUIDE.md** - Troubleshooting
- **README.md** - Code details

### **Code Comments:**

- Each behavior has detailed docstrings
- `competition_mission.py` has inline comments
- Launch scripts have explanation headers

### **Testing:**

```bash
# Test individual components
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /camera/image_raw/compressed
ros2 topic echo /color_sensor/rgb

# Test navigation
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"

# Check nodes
ros2 node list
ros2 node info /amcl
```

---

## 🏆 Ready to Win!

You now have everything needed:

- ✅ Split architecture (efficient)
- ✅ Full autonomy (no human input)
- ✅ Simple launch (3 commands)
- ✅ Comprehensive docs (5 guides)
- ✅ Proven approach (based on reference code)

**Next step**: Test on real robot! 🚀

---

**Quick Links:**

- Start: [QUICK_START.md](QUICK_START.md)
- **NEW**: [UPDATE_SUMMARY.md](UPDATE_SUMMARY.md) ← What changed!
- **NEW**: [FIELD_MAP.md](FIELD_MAP.md) ← Field coordinates
- Comparison: [PYTREES_COMPARISON.md](PYTREES_COMPARISON.md)
- Deploy: [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)
- Understand: [ARCHITECTURE_EXPLAINED.md](ARCHITECTURE_EXPLAINED.md)
- Visualize: [VISUAL_DIAGRAMS.md](VISUAL_DIAGRAMS.md)
- Reference: [README.md](README.md)

**Good luck! 🍀**
