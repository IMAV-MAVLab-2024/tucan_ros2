import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8

class ModeVerticalGate(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_vertical_gate')
        self.__own_state = 4
        self.state_subsciber = self.create_subscription(UInt8,'mission_state', self.__listener_callback,1)
        
        
    def execute(self):
        """Execute the vertical gate mode.
        """
        self.get_logger().info('Executing vertical gate mode')

        # Get the sample location in camera frame
        
        # 
        
    def __listener_callback(self, msg):
        if msg.data == self.__own_state:
            self.execute()
