import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from image_transport import ImageTransport

class DriverCamera(Node):
    """Camera driver node using image_transport"""
    def __init__(self):
        super().__init__('camera_driver_node')
        self.declare_parameter('camera_id', 22)  # Camera ID (e.g., 22, 31, 0)
        self.declare_parameter('FPS', 15)
        self.declare_parameter('frame_width', 800)
        self.declare_parameter('frame_height', 600)
        self.declare_parameter('topic_name', "/camera_image")
        self.declare_parameter('use_compressed', False)  # Whether to use compressed image transport

        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.FPS = self.get_parameter('FPS').get_parameter_value().integer_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value

        self.get_logger().info('Starting camera driver node with the following parameters:')
        self.get_logger().info('Camera ID: ' + str(self.camera_id))
        self.get_logger().info('FPS: ' + str(self.FPS))
        self.get_logger().info('Frame Width: ' + str(self.frame_width))
        self.get_logger().info('Frame Height: ' + str(self.frame_height))
        self.get_logger().info('Topic Name: ' + str(self.topic_name))
        self.get_logger().info('Using Compressed Transport: ' + str(self.use_compressed))

        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return

        if not self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width):
            self.get_logger().error('Failed to set frame width')
        if not self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height):
            self.get_logger().error('Failed to set frame height')
        if not self.cap.set(cv2.CAP_PROP_FPS, self.FPS):
            self.get_logger().error('Failed to set FPS')

        self.bridge = CvBridge()

        # Initialize ImageTransport
        self.image_transport = ImageTransport(self)

        # Select the publisher based on compression
        if self.use_compressed:
            self.image_publisher = self.image_transport.advertise(self.topic_name, "compressed", 1)
        else:
            self.image_publisher = self.image_transport.advertise(self.topic_name, "raw", 1)

        self.timer = self.create_timer(1/self.FPS, self.get_camera_images)
    
    def get_camera_images(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher.publish(image_msg)
        else:
            self.get_logger().error('Failed to capture frame')

def main():
    rclpy.init()
    camera_driver_node = DriverCamera()

    try:
        rclpy.spin(camera_driver_node)
    except KeyboardInterrupt:
        print("Shutting Down")
    rclpy.shutdown()
    camera_driver_node.destroy_node()

if __name__ == '__main__':
    main()
