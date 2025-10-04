#!/usr/bin/env python3
"""
Potato Disease Detection Node for Laptop
Subscribes to camera images from Raspberry Pi and performs disease classification
Supports both raw and compressed image formats for network efficiency
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
import time


class DiseaseDetectionNode(Node):
    def __init__(self):
        super().__init__('disease_detection_node')
        
        # Declare parameters
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('display_output', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('inference_rate', 1.0)  # Hz - don't process every frame
        self.declare_parameter('confidence_threshold', 0.0)  # Min confidence to report
        self.declare_parameter('model_name', 'model_ft_gpu.pth')
        
        # Get parameters
        use_compressed = self.get_parameter('use_compressed').value
        self.display = self.get_parameter('display_output').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        model_name = self.get_parameter('model_name').value
        
        self.bridge = CvBridge()
        
        # Rate limiting for inference
        self.last_inference_time = 0.0
        self.inference_interval = 1.0 / self.inference_rate if self.inference_rate > 0 else 0.0
        
        # Statistics
        self.frame_count = 0
        self.inference_count = 0
        self.last_result = "No detection yet"
        self.last_confidence = 0.0
        
        # Load ML model
        self.get_logger().info('🔄 Loading potato disease detection model...')
        try:
            from rpi_camera_package.common.inference_engine import PotatoDiseaseModel
            self.model = PotatoDiseaseModel(model_name=model_name)
            self.get_logger().info('✅ Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load model: {e}')
            self.get_logger().error('   Make sure PyTorch and torchvision are installed:')
            self.get_logger().error('   pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu')
            raise
        
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
        
        # Publishers
        self.result_pub = self.create_publisher(
            String,
            '/disease_detection/result',
            10
        )
        
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/disease_detection/annotated_image',
                10
            )
            self.get_logger().info('✅ Publishing annotated images')
        
        # Statistics timer
        self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('🚀 Disease Detection Node started on LAPTOP')
        self.get_logger().info(f'   Inference rate: {self.inference_rate} Hz')
        self.get_logger().info(f'   Confidence threshold: {self.confidence_threshold}')
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
        """Process image for disease detection with rate limiting"""
        try:
            # Rate limiting - don't process every frame
            current_time = time.time()
            if (current_time - self.last_inference_time) < self.inference_interval:
                # Skip this frame, but still display last result if enabled
                if self.display and self.last_result != "No detection yet":
                    annotated = self.create_annotated_image(
                        cv_image, self.last_result, self.last_confidence
                    )
                    cv2.imshow('Disease Detection - Laptop', annotated)
                    cv2.waitKey(1)
                return
            
            self.last_inference_time = current_time
            self.inference_count += 1
            
            # Convert OpenCV (BGR) to PIL (RGB)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Run inference
            class_name, confidence = self.model.predict(pil_image)
            
            # Check confidence threshold
            if confidence < self.confidence_threshold:
                return
            
            # Store last result
            self.last_result = class_name
            self.last_confidence = confidence
            
            # Create result message
            result_text = f"Disease: {class_name} | Confidence: {confidence*100:.1f}%"
            
            # Publish result string
            result_msg = String()
            result_msg.data = result_text
            self.result_pub.publish(result_msg)
            
            # Create and publish annotated image
            annotated_image = self.create_annotated_image(cv_image, class_name, confidence)
            
            if self.publish_annotated:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                annotated_msg.header = header
                self.annotated_pub.publish(annotated_msg)
            
            # Display if enabled
            if self.display:
                cv2.imshow('Disease Detection - Laptop', annotated_image)
                cv2.waitKey(1)
            
            # Log result with color coding
            if class_name == 'Healthy':
                self.get_logger().info(f'🟢 {result_text}')
            else:
                self.get_logger().warn(f'🔴 {result_text}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def create_annotated_image(self, cv_image, class_name, confidence):
        """
        Create annotated image with disease detection results.
        
        Args:
            cv_image: OpenCV BGR image
            class_name: Detected disease class
            confidence: Confidence score (0.0 to 1.0)
            
        Returns:
            Annotated OpenCV image
        """
        annotated = cv_image.copy()
        height, width = annotated.shape[:2]
        
        # Choose color based on disease status
        if class_name == 'Healthy':
            color = (0, 255, 0)  # Green
            status_emoji = "✓"
        elif class_name == 'Early_blight':
            color = (0, 165, 255)  # Orange
            status_emoji = "⚠"
        else:  # Late_blight
            color = (0, 0, 255)  # Red
            status_emoji = "✗"
        
        # Add semi-transparent overlay at top
        overlay = annotated.copy()
        cv2.rectangle(overlay, (0, 0), (width, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)
        
        # Add disease status
        status_text = f"{status_emoji} {class_name.replace('_', ' ').title()}"
        cv2.putText(annotated, status_text, (20, 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        
        # Add confidence
        confidence_text = f"Confidence: {confidence*100:.1f}%"
        cv2.putText(annotated, confidence_text, (20, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Add frame indicator in corner
        cv2.putText(annotated, f"Frame: {self.frame_count}", (width - 200, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        # Add bounding box around image
        cv2.rectangle(annotated, (5, 5), (width-5, height-5), color, 3)
        
        return annotated
    
    def print_stats(self):
        """Print periodic statistics"""
        if self.frame_count > 0:
            fps = self.frame_count / 5.0
            inference_fps = self.inference_count / 5.0
            self.get_logger().info(
                f'📊 Stats: {self.frame_count} frames, '
                f'{self.inference_count} inferences in 5s '
                f'(~{fps:.1f} FPS, ~{inference_fps:.1f} Inf/s)'
            )
        else:
            self.get_logger().info('📊 Stats: No frames received yet')
        
        self.frame_count = 0
        self.inference_count = 0
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DiseaseDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down disease detection node...')
    except Exception as e:
        print(f'\n❌ Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
