import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8

class ModePlaceSample(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_place_sample')
        self.__own_state = 7
        self.state_subsciber = self.create_subscription(UInt8,'mission_state', self.__listener_callback,1)
        
        
    def execute(self):
        """Execute the pick up mode.
        """
        self.get_logger().info('Executing pick up mode')

        # Get the sample location in camera frame
        
        # 
        
    def __listener_callback(self, msg):
        if msg.data == self.__own_state:
            self.execute()
