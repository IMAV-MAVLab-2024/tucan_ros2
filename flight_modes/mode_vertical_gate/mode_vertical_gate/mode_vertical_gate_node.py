import rclpy
from rclpy.node import Node

from tucan_msgs.msg import Mode, ModeStatus
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

class ModeVerticalGate(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_vertical_gate')
        self.mode = 4 # Gate mode ID is 4, DON'T CHANGE
        self.gate = 'top' # Specify gate 'top', 'middle', or 'bottom'
        self.state_subscriber = self.create_subscription(Mode,'/mission_state', self.__listener_callback,1)
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "/mode_status", 10)
        self.is_active = False
        
        self.control_mode_publisher_ = self.create_publisher(Mode, "/control_mode", 10)
        self.setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/trajectory_setpoint", 10)
        
        self.frequency = 1. # Node frequency in Hz
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
        match self.gate:
            case 'top':
                self.target_altitude = 2.1225 # Altitude of the top gate in meters
            case 'middle':
                self.target_altitude = 1.75 # Altitude of the middle gate in meters
            case 'bottom':
                self.target_altitude = 1.0 # Altitude of the bottom gate in meters
        
        
    def execute(self):
        """Execute the vertical gate mode.
        """
        

        # Task implementation
        
    def __listener_callback(self, msg):
        if msg.mode_id == self.mode:
            self.get_logger().info('Executing vertical gate mode')
            self.is_active = True
    
    def timer_callback(self):
        self.publish_mode_status()
        if self.is_active:
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
    mode_vertical_gate = ModeVerticalGate()

    rclpy.spin(mode_vertical_gate)

    mode_vertical_gate.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
