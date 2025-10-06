# Color Sensor Fix - Conditional Polling

## Problem

The color sensor was generating warning messages every 200ms (5Hz) when reporting low values:

```
[WARN] Color values very low: R=10 G=17 B=16. Check sensor wiring, power, or Arduino 'v' command response
```

This flooded the terminal and made debugging difficult.

## Solution Implemented

Changed the `read_color_sensor()` function to **only poll the sensor when there are active subscribers**:

```python
def read_color_sensor(self):
    """Read color sensor data periodically (only when there are active subscribers)"""
    if not self.serial or not self.serial.is_open:
        return

    # Only poll color sensor if someone is subscribed to the topics
    # This prevents unnecessary serial traffic and warning spam
    if (self.color_sensor_pub.get_subscription_count() == 0 and
        self.color_sensor_raw_pub.get_subscription_count() == 0):
        return

    # Send color read command...
```

## Benefits

1. **Clean terminal output** - No warnings when color sensor not in use
2. **Reduced serial traffic** - Doesn't poll Arduino unnecessarily
3. **Automatic activation** - Sensor activates as soon as you subscribe
4. **Lower CPU usage** - Less processing when sensor not needed

## Usage

### Normal operation (no color sensor polling)

```bash
./01_arduino_only.sh
# Terminal is clean, no color warnings
```

### When you need color data

```bash
# In another terminal:
ros2 topic echo /color_sensor/rgb
# Sensor automatically starts polling and publishing data
```

### Color detection service

```bash
ros2 service call /detect_color std_srvs/srv/Trigger
# Service temporarily enables LED and reads multiple samples
```

## Topics

- `/color_sensor/rgb` - Normalized ColorRGBA (0.0-1.0)
- `/color_sensor/raw` - Raw sensor values (String format)

## Notes

- Warning throttling (10 seconds) is still in place when sensor IS active
- This follows ROS 2 best practices for conditional sensor polling
- Service-based color detection still works independently (uses its own polling)
