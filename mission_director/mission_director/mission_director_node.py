import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8

class MissionDirector(Node):
    def __init__(self):
        super().__init__('mission_director')
        self.state_publisher = self.create_publisher(UInt8, 'mission_state', 10)
        

    def process(self):
        
        
        
def main(args=None):
    rclpy.init(args=args)
    
    mission_director = MissionDirector()
    
    rclpy.spin(mission_director)
    
    mission_director.destroy_node()
    rclpy.shutdown()