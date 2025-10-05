#!/usr/bin/env python3
"""
Color Detection Node for Laptop
Subscribes to camera images from Raspberry Pi and performs color detection
Supports both raw and compressed image formats for network efficiency
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        # Declare parameters
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('min_contour_area', 1000)
        self.declare_parameter('display_output', True)
        self.declare_parameter('publish_processed', True)
        
        # Get parameters
        use_compressed = self.get_parameter('use_compressed').value
        self.min_area = self.get_parameter('min_contour_area').value
        self.display = self.get_parameter('display_output').value
        self.publish_processed = self.get_parameter('publish_processed').value
        
        self.bridge = CvBridge()
        
        # QoS Profile for network communication
        # BEST_EFFORT is better for real-time streaming over network
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe based on compression preference
        if use_compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                '/camera/image_raw/compressed',
                self.compressed_callback,
                qos_profile
            )
            self.get_logger().info('✅ Subscribed to COMPRESSED images from Pi')
        else:
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                qos_profile
            )
            self.get_logger().info('✅ Subscribed to RAW images from Pi')
        
        # Publisher for processed image (optional)
        if self.publish_processed:
            self.processed_pub = self.create_publisher(
                Image, 
                '/color_detection/processed_image', 
                10
            )
            self.get_logger().info('✅ Publishing processed images')
        
        # Define color ranges in HSV format
        self.color_ranges = {
            'red': ([0, 120, 70], [10, 255, 255]),
            'green': ([40, 40, 40], [70, 255, 255]),
            'blue': ([110, 50, 50], [130, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255]),
        }
        
        # Statistics
        self.frame_count = 0
        self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('🚀 Color Detection Node started on LAPTOP')
        self.get_logger().info(f'   Min contour area: {self.min_area} pixels')
        self.get_logger().info(f'   Display window: {"ON" if self.display else "OFF"}')

    def compressed_callback(self, msg):
        """Handle compressed images from network"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                self.frame_count += 1
                self.process_image(cv_image, msg.header)
            else:
                self.get_logger().warn('Failed to decode compressed image')
                
        except Exception as e:
            self.get_logger().error(f'Error decompressing image: {e}')

    def image_callback(self, msg):
        """Handle raw images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            self.process_image(cv_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_image(self, cv_image, header):
        """Process image for color detection"""
        try:
            # Perform color detection
            processed_image, detected_colors = self.detect_colors(cv_image)
            
            # Publish processed image if enabled
            if self.publish_processed:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
                processed_msg.header = header
                self.processed_pub.publish(processed_msg)
            
            # Log detected colors
            if detected_colors:
                colors_str = ", ".join([f"✓ {c}" for c in detected_colors])
                self.get_logger().info(f'Detected colors: {colors_str}')
            
            # Display if enabled
            if self.display:
                cv2.imshow('Color Detection - Laptop', processed_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_colors(self, image):
        """
        Detect colors in the image and return processed image with detections
        
        Args:
            image: BGR image from camera
            
        Returns:
            tuple: (processed_image, list of detected color names)
        """
        # Convert to HSV color space (better for color detection)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        detected_colors = []
        processed_image = image.copy()
        
        # Check each color range
        for color_name, (lower, upper) in self.color_ranges.items():
            # Create color range mask
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process each contour
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Only consider contours above minimum area threshold
                if area > self.min_area:
                    # Draw bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Add label with color name
                    label = f"{color_name.upper()}"
                    cv2.putText(processed_image, label, (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Add area info
                    area_text = f"{int(area)} px"
                    cv2.putText(processed_image, area_text, (x, y + h + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    
                    # Add to detected colors list (avoid duplicates)
                    if color_name not in detected_colors:
                        detected_colors.append(color_name)
        
        # Add info overlay
        self.add_info_overlay(processed_image, detected_colors)
        
        return processed_image, detected_colors
    
    def add_info_overlay(self, image, detected_colors):
        """Add information overlay to the image"""
        # Add semi-transparent background for text
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (300, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, image, 0.5, 0, image)
        
        # Add text
        cv2.putText(image, "Color Detection - Laptop", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, f"Frame: {self.frame_count}", (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(image, f"Colors: {len(detected_colors)}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def print_stats(self):
        """Print periodic statistics"""
        fps = self.frame_count / 5.0
        self.get_logger().info(f'📊 Stats: {self.frame_count} frames in 5s (~{fps:.1f} FPS)')
        self.frame_count = 0
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down color detection node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
