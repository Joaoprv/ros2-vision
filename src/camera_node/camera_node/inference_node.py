import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

from ament_index_python.packages import get_package_share_directory
from camera_node.utils.yolov4_inference import YOLOv4

class YOLOv4Node(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.bridge = CvBridge()

        model_path = os.path.join(
            get_package_share_directory('ros2_vision'),
            'models', 'yolov4.onnx'
        )
        print(f"Model path: {model_path}")
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model file not found: {model_path}")
        self.detector = YOLOv4(model_path)

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.detector.infer(image)
        self.get_logger().info(f"Detections: {results}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv4Node()
    rclpy.spin(node)
    rclpy.shutdown()
