import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from .inference_engine import PotatoDiseaseModel
from PIL import Image as PILImage
import cv2



model = PotatoDiseaseModel()

class PotatoDiseaseDetection(Node):
    def __init__(self):
        super().__init__('potato_disease_detection_node')

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/inference_result', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Potato Disease Detection Node has started.")


    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert OpenCV image (BGR) to PIL image (RGB)
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

        # Run inference
        result = run_inference(pil_image)

        # Publish inference result
        result_msg = String()
        result_msg.data = result
        self.publisher_.publish(result_msg)

        self.get_logger().info(f'Published result: {result}')


    def run_dummy_inference(self, image: np.ndarray) -> str:
        """
        Placeholder ML inference for testing flow.
        """
        # Dummy classifier: detects if average pixel intensity > 127
        avg_intensity = np.mean(image)
        if avg_intensity > 127:
            return "Healthy Potato"
        else:
            return "Diseased Potato"

def run_inference(pil_image) -> str:
        """
        Run actual ML inference using the PotatoDiseaseModel.
        """

        result = model.predict(pil_image)

        print(f"Inference result: {result}")
        return result



def main(args=None):
    rclpy.init(args=args)
    node = PotatoDiseaseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
