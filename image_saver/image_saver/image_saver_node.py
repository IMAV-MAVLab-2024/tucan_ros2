import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/down_camera_image',  # Replace with your image topic name
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def get_next_free_image_name(self, directory, base_name='image', extension='.png'):
        """Find the next available image name in the given directory."""
        index = 1
        while True:
            file_name = f'{base_name}{index}{extension}'
            file_path = os.path.join(directory, file_name)
            if not os.path.exists(file_path):
                return file_path
            index += 1

    def listener_callback(self, msg):
        self.get_logger().info('Received an image, saving...')
        
        # Convert the ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        
        # Determine the next free image name in the home directory
        home_directory = os.path.expanduser('~/cv_images')  # Path to the home directory
        image_path = self.get_next_free_image_name(home_directory)

        # Save the image as a JPEG file
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'Image saved as {image_path}')
        
        # Shut down the node after saving the image
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()

    # Spin the node until it's shut down
    rclpy.spin(node)


if __name__ == '__main__':
    main()
