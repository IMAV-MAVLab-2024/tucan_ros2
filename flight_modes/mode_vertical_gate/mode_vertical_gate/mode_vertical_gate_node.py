import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from tucan_msgs.msg import Mode

class ModeVerticalGate(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_vertical_gate')
        self.__own_state = 4
        self.state_subsciber = self.create_subscription(Mode,'mission_state', self.__listener_callback,1)
        
        
    def execute(self):
        """Execute the vertical gate mode.
        """
        self.get_logger().info('Executing vertical gate mode')

        # Task implementation
        
    def __listener_callback(self, msg):
        if msg.data == self.__own_state:
            self.execute()
            
            self.__publish_finished()
    
    def __publish_finished(self):
        msg = Bool()
        msg.data = True
        self.finished_publsher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    mode_vertical_gate = ModeVerticalGate()

    rclpy.spin(mode_vertical_gate)

    mode_vertical_gate.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
