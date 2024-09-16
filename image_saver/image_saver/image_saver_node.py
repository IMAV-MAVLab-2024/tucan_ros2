import rclpy
from rclpy.node import Node
import rclpy.wait_for_message
from sensor_msgs.msg import Image
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        
        self.subscription_take_photo = self.create_subscription(
            std_msgs.Bool,
            '/take_photo',  # Replace with your image topic name
            self.take_photo,
            10)

    def get_next_free_image_name(self, directory, base_name='image', extension='.png'):
        """Find the next available image name in the given directory."""
        index = 1
        while True:
            file_name = f'{base_name}{index}{extension}'
            file_path = os.path.join(directory, file_name)
            if not os.path.exists(file_path):
                return file_path
            index += 1

    def take_photo(self, msg):
        if msg.data:
            self.get_logger().info('Taking a photo...')

            is_success, img_msg = rclpy.wait_for_message.wait_for_message(Image, self, '/down_camera_image', time_to_wait=2.0)
            if is_success:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                
                # Determine the next free image name in the home directory
                home_directory = os.path.expanduser('~/cv_images')  # Path to the home directory
                image_path = self.get_next_free_image_name(home_directory)

                # Save the image as a JPEG file
                cv2.imwrite(image_path, cv_image)
                self.get_logger().info(f'Image saved as {image_path}')
            else:
                self.get_logger().error('Failed to take a photo: no image received')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()

    # Spin the node until it's shut down
    rclpy.spin(node)


if __name__ == '__main__':
    main()