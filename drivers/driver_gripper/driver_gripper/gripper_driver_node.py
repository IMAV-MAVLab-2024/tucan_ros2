import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8

import tucan_msgs.msg as tucan_msgs

    

class DriverGripper(Node):
    """Gripper driver node
    """
    def __init__(self):
        super.__init__('driver_gripper')
        self.gripper_status_publisher = self.create_publisher(tucan_msgs.GripperStatus, 'gripper_status', 1)
        
        self.command_gripper = self.create_subscription(tucan_msgs.GripperCommand,"cmd_gripper", self.__listener_callback, 1)
        
        # settings
        self.frequency = 5

        self.status = tucan_msgs.GripperStatus()
        self.status.status = tucan_msgs.GripperStatus.OPENED

        self.timer = self.create_timer(1/self.frequency, self.__publish_status)

        # Clutch and continuous control settings
        us_clutch_engaged = 1650
        us_clutch_disengaged = 1350

        us_cont_rollup = 1028
        us_cont_rolloff = 1978
        us_cont_stop = 2928

        rollup_duration = 3.5
        rolloff_duration = 0.25
    
    def __publish_status(self):
        self.gripper_status_publisher.publish(self.status)

    def __listener_callback(self, msg):
        if msg.command == tucan_msgs.GripperCommand.OPEN:
            if self.status.status == tucan_msgs.GripperStatus.CLOSED:
                self.__gripper_open()
                self.status.status = tucan_msgs.GripperStatus.OPENING
            else:
                self.get_logger().info('Gripper not in closed state, cannot open')
        elif msg.command == tucan_msgs.GripperCommand.CLOSE:
            if self.status.status == tucan_msgs.GripperStatus.OPENED:
                self.__gripper_close()
                self.status.status = tucan_msgs.GripperStatus.CLOSING
            else:
                self.get_logger().info('Gripper not in open state, cannot close')
    
    def __gripper_start_open(self):
        

        
    
    def __gripper_close(self):
    
def main(args=None):
    rclpy.init(args=args)
    gripper_driver_node = DriverGripper()

    try:
        rclpy.spin(gripper_driver_node)
    except KeyboardInterrupt:
        pass
    
    gripper_driver_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()