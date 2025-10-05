#!/usr/bin/env python3
"""
Alternative camera node using rpicam-vid with ROS2 publisher
This works better than v4l2_camera for Raspberry Pi Camera Module 2

This node:
1. Captures video using rpicam-vid (modern libcamera)
2. Pipes raw frames to this script
3. Publishes as ROS2 compressed images

Requires: rpicam-apps (libcamera)
Install: sudo apt-get install rpicam-apps
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Header
import subprocess
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge


class RPiCameraNode(Node):
    def __init__(self):
        super().__init__('rpi_camera_node')
        
        # Declare parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 15)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('camera_name', 'rpi_camera')
        
        # Get parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.camera_name = self.get_parameter('camera_name').value
        
        # Publishers
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Frame counter
        self.frame_count = 0
        
        # Start camera capture
        self.get_logger().info(f'Starting rpicam-vid at {self.width}x{self.height}@{self.framerate}fps')
        self.start_camera()
        
    def start_camera(self):
        """Start rpicam-vid process"""
        # rpicam-vid command to output raw RGB frames to stdout
        cmd = [
            'rpicam-vid',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.framerate),
            '--timeout', '0',  # Run forever
            '--codec', 'yuv420',  # Raw YUV output
            '--output', '-',  # Output to stdout
            '--nopreview',  # No preview window
        ]
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            
            # Start thread to read frames
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            self.get_logger().info('Camera started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {str(e)}')
            raise
    
    def capture_frames(self):
        """Capture frames from rpicam-vid stdout"""
        # Calculate frame size (YUV420 is 1.5 bytes per pixel)
        frame_size = self.width * self.height * 3 // 2
        
        while rclpy.ok():
            try:
                # Read one frame worth of data
                raw_frame = self.process.stdout.read(frame_size)
                
                if len(raw_frame) != frame_size:
                    self.get_logger().warn('Incomplete frame received')
                    continue
                
                # Convert YUV420 to RGB
                yuv = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
                bgr_frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
                
                # Publish frame
                self.publish_frame(bgr_frame)
                
            except Exception as e:
                self.get_logger().error(f'Error capturing frame: {str(e)}')
                break
    
    def publish_frame(self, frame):
        """Publish frame as compressed image"""
        try:
            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_optical_frame'
            
            # Compress image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, compressed_data = cv2.imencode('.jpg', frame, encode_param)
            
            # Create compressed image message
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = compressed_data.tobytes()
            
            # Publish
            self.image_pub.publish(msg)
            
            # Publish camera info
            self.publish_camera_info(header)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames (~2 seconds at 15fps)
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')
    
    def publish_camera_info(self, header):
        """Publish camera info"""
        info = CameraInfo()
        info.header = header
        info.height = self.height
        info.width = self.width
        info.distortion_model = 'plumb_bob'
        
        # Simple camera matrix (assuming no calibration)
        # You should calibrate your camera and replace these values
        info.k = [
            self.width, 0.0, self.width / 2.0,
            0.0, self.width, self.height / 2.0,
            0.0, 0.0, 1.0
        ]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            self.width, 0.0, self.width / 2.0, 0.0,
            0.0, self.width, self.height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.camera_info_pub.publish(info)
    
    def __del__(self):
        """Cleanup"""
        if hasattr(self, 'process'):
            self.process.terminate()
            self.process.wait()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RPiCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
