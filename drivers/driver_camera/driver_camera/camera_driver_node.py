import rclpy
from rclpy.node import Node
import cv2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DriverCamera(Node):
    """Camera driver node"""
    def __init__(self):
        super().__init__('camera_driver_node')
        # Default for laptop testing
        self.declare_parameter('camera_type', 'laptop')
        which_camera = self.get_parameter('camera_type').get_parameter_value().string_value
        # print(which_camera)

        # Settings
        self.FPS= 15
        self.frame_width = 800
        self.frame_height = 600

        if which_camera == 'front':
            self.cap = cv2.VideoCapture(31)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.FPS)
            self.image_publisher = self.create_publisher(Image, 'front_camera_image', self.FPS)
            self.get_logger().info('Opening front camera')
        elif which_camera == 'down':  
            self.cap = cv2.VideoCapture(22)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.FPS)
            self.image_publisher = self.create_publisher(Image, 'down_camera_image', self.FPS)
            self.get_logger().info('Opening front camera')
        else:
            self.cap = cv2.VideoCapture(0)
            self.image_publisher = self.create_publisher(Image, 'laptop_camera_image', self.FPS)
        self.bridge_for_CV = CvBridge()

        self.timer = self.create_timer(1/self.FPS, self.get_camera_images)
    
    def get_camera_images(self):
        # Get camera image
        ret, frame = self.cap.read()     
        if ret == True:
            # Publish camera image
            self.image_publisher.publish(self.bridge_for_CV.cv2_to_imgmsg(frame))
            # encoding="passthrough"
        self.get_logger().debug('Publishing video frame')
            
def main():
    rclpy.init()
    camera_driver_node = DriverCamera()

    try:
        rclpy.spin(camera_driver_node)
    except KeyboardInterrupt:
         print("Shutting Down") 
    rclpy.shutdown()

    
    camera_driver_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()