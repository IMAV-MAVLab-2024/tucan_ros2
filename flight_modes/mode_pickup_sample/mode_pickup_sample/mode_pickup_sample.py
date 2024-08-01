import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8, Bool

class ModePickupSample(Node):
    
        def __init__(self):
            super().__init__('mode_pickup_sample')
            
            # flight mode finished publisher; publish 1 to hand control to mission director
            self.fm_finish_publisher = self.create_publisher(Bool, 'mode_finished', 1)
            
            self.state_subscriber = self.create_subscription(UInt8, 'mission_state', self.__state_callback, 10)
            
            self.is_active = False
        
        def run(self):
            if self.is_active:
                self.execute()
                self.is_active = False


        def execute(self):
            # Code implementation
            pass

        def __state_callback(self):
            self.is_active = True
            
def main(args=None):
    rclpy.init(args=args)
    mode_pickup_sample = ModePickupSample()

    try:
        mode_pickup_sample.run()
    except KeyboardInterrupt:
        pass
    
    mode_pickup_sample.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()