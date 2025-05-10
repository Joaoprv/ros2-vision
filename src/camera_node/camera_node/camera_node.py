import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Default camera device
        self.declare_parameter('camera_device', '/dev/video0')
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Open the camera device
        self.cap = cv2.VideoCapture(self.camera_device)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera: {self.camera_device}")
            exit(1)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz publishing rate

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing image')
        else:
            self.get_logger().error("Failed to capture image")

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
