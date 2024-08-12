import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from tucan_msgs.msg import Mode

class ModeWildlifePhotographer(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_wildlife_photographer')
        self.__own_state = 3
        self.state_subscriber = self.create_subscription(Mode,'mission_state', self.__listener_callback,1)
        self.finished_publsher = self.create_publisher(Bool,'mode_finished', 10)
        
        self.frequency = 20 # Node frequency in Hz
        
    def execute(self):
        """Execute the wildlife photographer mode.
        """
        self.get_logger().info('Executing wildlife photographer mode')

        # Implementation of the task

        
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
    mode_wildlife_photographer = ModeWildlifePhotographer()

    rclpy.spin(mode_wildlife_photographer)

    mode_wildlife_photographer.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()