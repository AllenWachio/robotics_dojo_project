#!/usr/bin/env python3
"""
Color Sensor Behavior Nodes
============================
Behaviors for reading RGB color sensor data from Arduino
Topics: /color_sensor/rgb (ColorRGBA), /color_sensor/led (Bool)

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import py_trees
from std_msgs.msg import ColorRGBA, Bool
import time


class ReadColorSensor(py_trees.behaviour.Behaviour):
    """
    Read color sensor and classify as red, blue, green, or unknown.
    
    Operation:
    1. Turn on LED for accurate color reading
    2. Wait for RGB values from /color_sensor/rgb topic
    3. Classify color based on RGB dominance
    4. Store result in blackboard['detected_color']
    5. Turn off LED
    
    Returns: Always SUCCESS (manual loading fallback on timeout)
    Blackboard Output: 'detected_color' = 'red'|'blue'|'green'|'unknown'
    """
    
    def __init__(self, name="ReadColorSensor", timeout=15.0):
        super().__init__(name)
        self.timeout = timeout
        self.node = None
        self.color_sub = None
        self.led_pub = None
        self.latest_color = None
        self.start_time = None
        self.blackboard = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 components"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        # Subscribe to color sensor RGB data
        self.color_sub = self.node.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.color_callback,
            10
        )
        
        # Publisher to control LED
        self.led_pub = self.node.create_publisher(Bool, '/color_sensor/led', 10)
        
        # Setup blackboard
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(
            key='detected_color',
            access=py_trees.common.Access.WRITE
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def color_callback(self, msg):
        """Store latest color reading"""
        self.latest_color = (msg.r, msg.g, msg.b)
    
    def initialise(self):
        """Start color reading"""
        self.latest_color = None
        self.start_time = time.time()
        
        # Turn on LED
        led_msg = Bool()
        led_msg.data = True
        self.led_pub.publish(led_msg)
        self.logger.info(f"{self.name}: 💡 LED ON, reading color sensor...")
    
    def update(self):
        """Classify color based on RGB values"""
        # Check timeout - allow manual intervention
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: ⏱️ Timeout - no color detected")
            self.logger.info(f"{self.name}: 🤚 Assuming manual loading, continuing...")
            self.blackboard.detected_color = 'unknown'
            self._turn_off_led()
            return py_trees.common.Status.SUCCESS
        
        # Wait for color data
        if self.latest_color is None:
            return py_trees.common.Status.RUNNING
        
        # Classify color
        r, g, b = self.latest_color
        
        # Normalize to 0-1 range (if needed)
        if max(r, g, b) > 1.0:
            r, g, b = r / 255.0, g / 255.0, b / 255.0
        
        # Color classification logic
        detected_color = 'unknown'
        
        # Red dominant
        if r > 0.5 and r > g * 1.5 and r > b * 1.5:
            detected_color = 'red'
        # Blue dominant
        elif b > 0.4 and b > r * 1.3 and b > g * 1.3:
            detected_color = 'blue'
        # Green dominant
        elif g > 0.4 and g > r * 1.3 and g > b * 1.3:
            detected_color = 'green'
        # Yellow (red + green)
        elif r > 0.4 and g > 0.4 and b < 0.3:
            detected_color = 'yellow'
        
        # Store result
        self.blackboard.detected_color = detected_color
        self.logger.info(f"{self.name}: ✓ Detected color: {detected_color.upper()} (R={r:.2f}, G={g:.2f}, B={b:.2f})")
        
        # Turn off LED
        self._turn_off_led()
        
        return py_trees.common.Status.SUCCESS
    
    def _turn_off_led(self):
        """Helper to turn off LED"""
        led_msg = Bool()
        led_msg.data = False
        self.led_pub.publish(led_msg)
        self.logger.info(f"{self.name}: 💡 LED OFF")
    
    def terminate(self, new_status):
        """Ensure LED is off"""
        self._turn_off_led()


class WaitForColorSensorClear(py_trees.behaviour.Behaviour):
    """
    Wait until color sensor no longer detects any object.
    Used to verify cube has been offloaded.
    
    Operation:
    - Monitor /color_sensor/rgb for low RGB values (no object present)
    - Returns SUCCESS when RGB values drop below threshold
    - Returns FAILURE on timeout (cube still present)
    
    Returns: SUCCESS (clear) | FAILURE (timeout - cube still there)
    """
    
    def __init__(self, name="WaitForColorSensorClear", timeout=5.0, threshold=0.2):
        super().__init__(name)
        self.timeout = timeout
        self.threshold = threshold  # Max RGB value for "clear"
        self.node = None
        self.color_sub = None
        self.latest_color = None
        self.start_time = None
        
    def setup(self, **kwargs):
        """Initialize ROS2 subscription"""
        self.node = kwargs.get('node')
        if not self.node:
            self.logger.error(f"{self.name}: No node provided!")
            return
        
        self.color_sub = self.node.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.color_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")
    
    def color_callback(self, msg):
        """Store latest color reading"""
        self.latest_color = (msg.r, msg.g, msg.b)
    
    def initialise(self):
        """Start monitoring"""
        self.latest_color = None
        self.start_time = time.time()
        self.logger.info(f"{self.name}: Waiting for color sensor to clear (no object)...")
    
    def update(self):
        """Check if sensor is clear"""
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            self.logger.warning(f"{self.name}: ⏱️ Timeout - cube may still be present")
            return py_trees.common.Status.FAILURE
        
        # Wait for data
        if self.latest_color is None:
            return py_trees.common.Status.RUNNING
        
        # Check if clear (all RGB values low)
        r, g, b = self.latest_color
        
        # Normalize if needed
        if max(r, g, b) > 1.0:
            r, g, b = r / 255.0, g / 255.0, b / 255.0
        
        max_value = max(r, g, b)
        
        if max_value < self.threshold:
            self.logger.info(f"{self.name}: ✓ Sensor clear (max RGB: {max_value:.3f})")
            return py_trees.common.Status.SUCCESS
        
        # Still detecting object
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Cleanup"""
        pass
