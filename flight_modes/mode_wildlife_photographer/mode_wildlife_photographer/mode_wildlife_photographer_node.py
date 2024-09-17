import rclpy
from rclpy.node import Node

from tucan_msgs.msg import Mode, ModeStatus

class ModeWildlifePhotographer(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_wildlife_photographer')
        self.mode = 3 # Photography mode ID is 3, DON'T CHANGE
        self.state_subscriber = self.create_subscription(Mode,'mission_state', self.__listener_callback,1)
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "mode_status", 10)
        self.is_active = False
        
        self.frequency = 1. # Node frequency in Hz
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
    def execute(self):
        """Execute the wildlife photographer mode.
        """
        self.get_logger().info('Executing wildlife photographer mode')

        # Implementation of the task

        
    def __listener_callback(self, msg):
        if msg.mode_id == self.mode:
            self.is_active = True
    
    def timer_callback(self):
        if self.is_active:
            self.publish_mode_status()
            self.is_active = False
            self.execute()
        
    def publish_mode_status(self):
        msg = ModeStatus()
        msg.mode.mode_id = self.mode
        if self.is_active:
            msg.mode_status = msg.MODE_ACTIVE
        else:
            msg.mode_status = msg.MODE_FINISHED
        msg.busy = self.is_active
        self.mode_status_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mode_wildlife_photographer = ModeWildlifePhotographer()

    rclpy.spin(mode_wildlife_photographer)

    mode_wildlife_photographer.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()