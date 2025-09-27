#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('color_detection_subscriber')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher for processed image (optional)
        self.processed_publisher_ = self.create_publisher(Image, '/color_detection/processed_image', 10)
        
        # Define color ranges (HSV format)
        self.color_ranges = {
            'red': ([0, 120, 70], [10, 255, 255]),
            'green': ([40, 40, 40], [70, 255, 255]),
            'blue': ([110, 50, 50], [130, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255]),
        }
        
        self.get_logger().info('Color detection subscriber started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image for color detection
            processed_image, detected_colors = self.detect_colors(cv_image)
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            processed_msg.header = msg.header
            self.processed_publisher_.publish(processed_msg)
            
            # Log detected colors
            if detected_colors:
                self.get_logger().info(f'Detected colors: {", ".join(detected_colors)}')
                
        except Exception as e:
            self.get_logger().error(f'Error in color detection: {e}')

    def detect_colors(self, image):
        """Detect colors in the image and return processed image with detections"""
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        detected_colors = []
        processed_image = image.copy()
        
        for color_name, (lower, upper) in self.color_ranges.items():
            # Create masks for color range
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:  # Only consider large enough areas
                    # Draw bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Add label
                    cv2.putText(processed_image, color_name, (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    if color_name not in detected_colors:
                        detected_colors.append(color_name)
        
        return processed_image, detected_colors

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetectionSubscriber()
    
    try:
        rclpy.spin(color_detector)
    except KeyboardInterrupt:
        pass
    finally:
        color_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
