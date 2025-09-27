#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import tempfile
import os
import numpy as np
import threading
import queue
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, "/camera/image_raw", 10)

        # Threading for better performance
        self.frame_queue = queue.Queue(maxsize=2)
        self.capture_thread = None
        self.running = True

        # Start capture thread
        self.start_capture_thread()

        # Timer for publishing
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 FPS target

        self.get_logger().info("Camera publisher node started (threaded version)")

    def start_capture_thread(self):
        """Start continuous capture in separate thread"""
        self.capture_thread = threading.Thread(target=self._continuous_capture)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def _continuous_capture(self):
        """Continuous capture in separate thread"""
        while self.running:
            try:
                frame = self.capture_with_rpicam()
                if frame is not None:
                    # Put frame in queue (non-blocking)
                    if not self.frame_queue.full():
                        self.frame_queue.put(frame, block=False)
                time.sleep(0.033)  # 30 FPS max
            except Exception as e:
                self.get_logger().error(f"Capture thread error: {e}")
                time.sleep(0.1)

    def capture_with_rpicam(self):
        """Capture frame using rpicam-still command"""
        try:
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_file:
                temp_filename = temp_file.name

            cmd = [
                "rpicam-still",
                "-o",
                temp_filename,
                "-t",
                "50",
                "--width",
                "640",
                "--height",
                "480",
                "--nopreview",
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)

            if result.returncode == 0 and os.path.exists(temp_filename):
                img = cv2.imread(temp_filename)
                os.unlink(temp_filename)
                return img

            if os.path.exists(temp_filename):
                os.unlink(temp_filename)

        except Exception as e:
            if "temp_filename" in locals() and os.path.exists(temp_filename):
                os.unlink(temp_filename)

        return None

    def timer_callback(self):
        try:
            # Get latest frame from queue
            if not self.frame_queue.empty():
                frame = self.frame_queue.get_nowait()

                # Publish the image
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_frame"
                self.publisher_.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

    def destroy_node(self):
        self.running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
