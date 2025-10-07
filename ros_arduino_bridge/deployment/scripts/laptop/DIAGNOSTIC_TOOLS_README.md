# Navigation Diagnostic Tools

This directory contains tools to help you diagnose and fix navigation issues, especially the critical problem where **Nav2 Goal doesn't publish to /cmd_vel**.

## 📋 Available Tools

### 1. **Full Diagnostics Script** ⭐ **START HERE**

```bash
./05_navigation_diagnostics.sh
```

**What it does:**

- ✅ Checks all Nav2 nodes are running
- ✅ Verifies TF transform chain (map→odom→base_link)
- ✅ Tests critical topics (/cmd_vel, /scan, /odom, /map)
- ✅ Checks topic data flow rates
- ✅ Verifies lifecycle states
- ✅ Tests costmap publishing
- ✅ **Tests /cmd_vel connectivity** (THE MOST IMPORTANT!)
- ✅ Checks robot localization status

**When to use:** Before EVERY navigation session to catch issues early!

**Output:** Clear pass/fail report with specific fixes for each issue

---

### 2. **Quick Diagnostics Reference**

```bash
./QUICK_DIAGNOSTICS.sh
```

**What it does:**

- Displays all diagnostic commands in one reference
- Copy-paste commands for specific checks
- Common fixes for each issue

**When to use:** Quick reference when you need specific commands

---

### 3. **Real-Time Monitor**

```bash
./06_realtime_monitor.sh
```

**What it does:**

- Live monitoring of navigation status
- Shows /cmd_vel velocity commands in real-time
- Tracks node status
- Updates every second

**When to use:** Run in separate terminal while navigating to watch what's happening

---

### 4. **Complete Troubleshooting Guide**

```
NAVIGATION_TROUBLESHOOTING_GUIDE.md
```

**What it contains:**

- Detailed explanations of every diagnostic command
- Step-by-step troubleshooting procedures
- Common issues and solutions
- Understanding navigation data flow
- Pro tips for debugging

**When to use:** When you need deep understanding or stuck on a specific issue

---

## 🚀 Quick Start Workflow

### **Every Time Before Navigating:**

1. **Launch your navigation stack:**

   ```bash
   cd ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop
   ./02c_slam_navigation_mode.sh
   ```

2. **Run diagnostics (in new terminal):**

   ```bash
   ./05_navigation_diagnostics.sh
   ```

3. **If all checks pass:**

   - Open RViz
   - Set initial pose (2D Pose Estimate)
   - Wait 3-5 seconds
   - Set navigation goal (Nav2 Goal)
   - Watch robot navigate!

4. **Optional: Monitor in real-time (new terminal):**
   ```bash
   ./06_realtime_monitor.sh
   ```

---

## 🔴 The Most Critical Check: /cmd_vel

**THE ISSUE YOU HAD:** Nav2 Goal was set but nothing published to /cmd_vel, so robot didn't move.

**How to verify it's working:**

```bash
# 1. Check topic exists
ros2 topic list | grep cmd_vel

# 2. Check subscribers (CRITICAL!)
ros2 topic info /cmd_vel
# MUST show "Subscription count: 1" or more
# If 0: Robot hardware not connected!

# 3. Test manual command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
# Robot should move forward slightly

# 4. Monitor during navigation
ros2 topic echo /cmd_vel
# Should see continuous updates when navigating
```

**If /cmd_vel has 0 subscribers:**

- ❌ Robot hardware NOT running on Pi
- Check: `ros2 node list | grep arduino`
- Fix: Launch `01_arduino_only.sh` on Pi

---

## 🎯 Common Issues Caught by Diagnostics

### Issue 1: "Nav2 nodes not running"

**Symptom:** Can't set goals in RViz  
**Fix:** Launch navigation: `./02c_slam_navigation_mode.sh`

### Issue 2: "TF map frame doesn't exist"

**Symptom:** Errors about missing map frame  
**Fix:** No localization running - launch navigation mode

### Issue 3: "/cmd_vel has 0 subscribers"

**Symptom:** Goals set but robot doesn't move  
**Fix:** Robot hardware not running - check Pi

### Issue 4: "No path generated"

**Symptom:** Goal accepted but no green line in RViz  
**Fix:** Set goal in free space, check map loaded

### Issue 5: "Lifecycle nodes inactive"

**Symptom:** Nav2 nodes exist but don't respond  
**Fix:** Check autostart:=true in launch file

---

## 📊 Understanding the Diagnostic Output

### ✅ Green Checkmarks = Good

All systems working correctly

### ⚠️ Yellow Warnings = Attention

Not critical but should be addressed

### ❌ Red X's = Critical

Must fix before navigation will work

---

## 💡 Pro Tips

1. **Always run diagnostics first** - Saves 10+ minutes of debugging
2. **Keep monitor running** - See exactly when /cmd_vel starts/stops
3. **Test /cmd_vel manually** - Confirms hardware connection
4. **Check subscriber count** - The #1 issue for navigation
5. **Verify TF chain** - Second most common issue
6. **Set initial pose carefully** - Close to actual position
7. **Start with short goals** - Test basic functionality first

---

## 🆘 Still Having Issues?

1. **Run full diagnostics and save output:**

   ```bash
   ./05_navigation_diagnostics.sh > my_diagnostics.txt
   ```

2. **Capture system state:**

   ```bash
   # Node list
   ros2 node list > nodes.txt

   # Topic list
   ros2 topic list > topics.txt

   # TF tree
   ros2 run tf2_tools view_frames.py
   ```

3. **Read the full guide:**

   ```bash
   cat NAVIGATION_TROUBLESHOOTING_GUIDE.md
   ```

4. **Check logs:**
   ```bash
   ros2 run rqt_console rqt_console
   ```

---

## 📚 Additional Resources

- **Complete workflow:** `../COMPLETE_WORKFLOW.md`
- **Navigation setup:** `NAVIGATION_SETUP.md`
- **Deployment guide:** `../IMPLEMENTATION_SUMMARY.md`

---

## ✨ Summary

**Before you had these tools:**

- ❌ Had to manually check everything
- ❌ No clear way to identify issues
- ❌ Didn't know /cmd_vel had no subscribers
- ❌ Spent time debugging blind

**Now with these tools:**

- ✅ Automated comprehensive checks
- ✅ Clear identification of issues
- ✅ Specific fixes for each problem
- ✅ Real-time monitoring
- ✅ Catch issues before they cause problems

**Run `./05_navigation_diagnostics.sh` before every navigation session!** 🎯
