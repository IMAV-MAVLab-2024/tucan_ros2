import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8, Bool
from tucan_msgs.msg import Mode

class ModePickupSample(Node):
    """Sample pick up mode node.
    """
    def __init__(self):
        super().__init__('mode_pickup_sample')
        self.__own_state = 6
        self.state_subsciber = self.create_subscription(Mode,'mode_finished', self.__listener_callback,1)
        
        
    def activate(self):
        """Execute the pick up mode.
        """
        self.get_logger().info('Executing pick up mode')

        # Get the sample location in camera frame
        
        # 
        
    def __listener_callback(self, msg):
        if msg.mode == self.__own_state:
            self.activate()
