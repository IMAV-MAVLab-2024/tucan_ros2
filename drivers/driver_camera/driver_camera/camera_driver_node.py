import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class DriverCamera(Node):
    """Camera driver node
    """
    def __init__(self):
        super.__init__('driver_camera')
        self.front_image_publisher = self.create_publisher(Image, 'front_camera_image', 1)
        self.down_image_publisher = self.create_publisher(Image, 'down_camera_image', 1)
        
        
        # Settings
        self.frequency = 20
    
    def run(self):
        """Run the camera driver node
        """
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            self.__get_camera_images()
            rate.sleep()

    def __get_camera_images(self):
        
        # Get front camera image
        # Publish front camera image
        
        # Get down camera image
        # Publish down camera image
        pass
        
def main(args=None):
    rclpy.init(args=args)
    camera_driver_node = DriverCamera()

    try:
        camera_driver_node.run()
    except KeyboardInterrupt:
        pass
    
    camera_driver_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()