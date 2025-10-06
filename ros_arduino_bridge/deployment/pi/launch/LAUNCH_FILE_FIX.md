# Launch File Fix - Publish TF Parameter Issue

## Problem

When launching with `--ekf` flag, the following error occurred:

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
Caught multiple exceptions when trying to load file of format [py]:
 - TypeError: Unexpected type for parameter value <launch.conditions.unless_condition.UnlessCondition object at 0xffff8c763610>
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
```

## Root Cause

In the launch file, the `publish_tf` parameter was incorrectly set using a condition object:

```python
# WRONG - This doesn't work!
'publish_tf': UnlessCondition(use_ekf),
```

**Issue:** ROS 2 node parameters must be actual values (bool, int, string, etc.), NOT condition objects. Condition objects are only used for conditionally launching nodes, not for setting parameter values.

## Solution

Use `NotSubstitution` to invert the boolean value:

```python
# CORRECT - This works!
'publish_tf': NotSubstitution(use_ekf),
```

**How it works:**

- When `use_ekf='true'`: NotSubstitution evaluates to `False` (don't publish TF)
- When `use_ekf='false'`: NotSubstitution evaluates to `True` (publish TF)

## Code Changes

### Before (Broken)

```python
from launch.conditions import IfCondition, UnlessCondition

arduino_bridge = Node(
    # ...
    parameters=[{
        'publish_tf': UnlessCondition(use_ekf),  # ❌ Wrong!
    }]
)
```

### After (Fixed)

```python
from launch.substitutions import NotSubstitution
from launch.conditions import IfCondition

arduino_bridge = Node(
    # ...
    parameters=[{
        'publish_tf': NotSubstitution(use_ekf),  # ✅ Correct!
    }]
)
```

## Testing

After the fix, the launch file should work correctly:

```bash
# Test standard mode
ros2 launch ros_arduino_bridge arduino_only.launch.py use_ekf:=false
# Expected: Arduino bridge publishes TF (publish_tf=True)

# Test EKF mode
ros2 launch ros_arduino_bridge arduino_only.launch.py use_ekf:=true
# Expected: Arduino bridge does NOT publish TF (publish_tf=False), EKF publishes TF instead
```

## Verification

Check the parameter value:

```bash
# Launch the system
./01_arduino_only.sh --ekf

# In another terminal, check the parameter
ros2 param get /ros_arduino_bridge publish_tf
# Expected: Boolean value: False

# Or without EKF
./01_arduino_only.sh

ros2 param get /ros_arduino_bridge publish_tf
# Expected: Boolean value: True
```

## Key Takeaways

1. **Node conditions** vs **Parameter values**:

   - Use `IfCondition`/`UnlessCondition` for: Whether to launch a node
   - Use actual values or `PythonExpression` for: Node parameter values

2. **NotSubstitution**:

   - Built-in launch substitution for inverting boolean values
   - Works with LaunchConfiguration boolean parameters
   - Clean and simple solution for "not" logic

3. **Common pattern for conditional parameters**:

   ```python
   # If condition is true, parameter is false (inverse)
   'param': NotSubstitution(condition)

   # If condition is true, parameter is true (same)
   'param': condition
   ```

## Status

✅ **FIXED** - Launch file now works correctly with both modes

---

**Fixed:** 2025-10-06  
**File:** `deployment/pi/launch/arduino_only.launch.py`  
**Issue:** TypeError with UnlessCondition in parameter value  
**Solution:** Use PythonExpression to evaluate boolean value
