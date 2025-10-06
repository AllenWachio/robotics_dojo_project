import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        timer_period = 2.0  # publish every 2 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # Test Image
        cv_image = cv2.imread(
            './src/rdj2025_potato_disease_detection/rdj2025_potato_disease_detection/healthy.jpg')
        if cv_image is None:
            self.get_logger().error("Could not read image")
            return

        # Publish the image
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published test image.')


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
