import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8
    

class DriverGripper(Node):
    """Gripper driver node
    """
    def __init__(self):
        super.__init__('driver_gripper')
        self.gripper_status_publisher = self.create_publisher(UInt8, 'gripper_status', 1)
        
        self.command_gripper = self.create_subscription(UInt8,"cmd_gripper", self.__listener_callback, 1)
        
        # settings
        self.frequency = 20

    
    def run(self):
        """Run the camera driver node
        """
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            self.__get_camera_images()
            rate.sleep()

    def __listener_callback(self, msg):
        
        if msg.data == 1:
            self.__gripper_open()
        
        elif msg.data == 2:
            self.__gripper_close()
    
    def __gripper_open(self):
        # Open the gripper
        pass
    
    def __gripper_close(self):
        # Close the gripper
        pass
        
def main(args=None):
    rclpy.init(args=args)
    gripper_driver_node = DriverGripper()

    try:
        gripper_driver_node.run()
    except KeyboardInterrupt:
        pass
    
    gripper_driver_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()