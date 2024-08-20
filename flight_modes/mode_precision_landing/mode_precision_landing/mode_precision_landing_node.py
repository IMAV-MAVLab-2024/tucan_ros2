import rclpy
from rclpy.node import Node

from tucan_msgs.msg import Mode, ModeStatus, ARMarker
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode

class ModePrecisionLanding(Node):
    """Flight mode for landing on an AR marker
    """
    def __init__(self):
        super().__init__('mode_precision_landing')
        self.mode = 5 # Precision landing mode ID is 5, DON'T CHANGE
        self.state_subscriber = self.create_subscription(Mode,'mission_state', self.__listener_callback,1)
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "mode_status", 10)
        self.is_active = False
        
        self.frequency = 1. # Node frequency in Hz
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
        self.ar_subscriber = self.create_subscription(ARMarker, 'cv_aruco_detector', self.__ar_callback, 10)
        self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/trajectory_setpoint', 10)
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/control_mode', 10)
        self.__landing_speed = -0.5 # m/s, downward vertical speed
        
        self.AR_x_offset = 0.
        self.AR_y_offset = 0.
        
        # settings
        self.forward_velocity_gain = 0.5
        self.sideways_velocity_gain = 0.5
        self.x_px = 800
        self.y_px = 600
        
    def execute(self):
        """Execute precision landing mode.
        Set self.is_active to False when finished.
        """
        self.get_logger().info('Executing precision landing mode')

        # Keep marker in the middle
        
    def __ar_callback(self, msg):
        """Set AR marker offsets on node class
        """
        # offsets are between -0.5 and 0.5 and flipped to the FRD frame
        self.AR_y_offset = float(msg.x)/self.x_px - 0.5
        self.AR_x_offset = float(msg.y)/self.y_px - 0.5
        
        if msg.id == 0:
            self.AR_x_offset = 0.
            self.AR_y_offset = 0.
        
    def __listener_callback(self, msg):
        if msg.data == self.mode:
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

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        forward_velocity = self.forward_velocity_gain * self.AR_x_offset
        sideways_velocity = self.sideways_velocity_gain * self.AR_y_offset
        msg.velocity = {forward_velocity, sideways_velocity, self.__landing_speed}
        msg.yaw = 0.0
        self.setpoint_publisher.publish(msg)
    
    def publish_offboard_position_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mode_precision_landing = ModePrecisionLanding()

    rclpy.spin(mode_precision_landing)

    mode_precision_landing.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()