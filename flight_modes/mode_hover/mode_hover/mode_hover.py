import rclpy
from rclpy.node import Node

from px4_msgs import TrajectorySetpoint, OffboardControlMode
from tucan_msgs import Mode, ModeStatus, ARMarker

class ModeHover(Node):
    """flight mode to hover above an AR marker.
    Runs a timer that publishes the mode status every 1/frequency seconds and if node is set to active, publishes velocity setpoints.
    Has a small internal control loop that adjusts the velocity setpoints based on the AR marker position.
    """
    def __init__(self):
        super().__init__('mode_hover')
        self.get_logger().info('ModeHover initialized')
        self.mode = 1 # Idle mode ID is 0
        self.__frequency = 20 # Frequency in Hz
        
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(Mode, "/mission_state", self.state_callback, 10)
        
        self.setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/trajectory_setpoint", 10)
        self.control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/offboard_control_mode", 10)
        
        self.AR_subsciber_ = self.create_subscription(ARMarker, "/cv_ar_detection", self.AR_callback, 10)
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
        self.AR_x_offset = 0.
        self.AR_y_offset = 0.
        
    def timer_callback(self):
        """Timer callback executes every 1/frequency seconds
        Publishes node status and if node is set active, publishes trajectory setpoints and velocity control mode
        """
        self.publish_mode_status()
        if self.is_active:
            self.publish_trajectory_setpoint()
            self.publish_offboard_position_mode()
        
    def publish_mode_status(self):
        msg = ModeStatus()
        msg.mode = self.mode
        if self.is_active:
            msg.mode_status = msg.MODE_ACTIVE
        else:
            msg.mode_status = msg.MODE_FINISHED
        msg.busy = self.is_active
        self.mode_status_publisher_.publish(msg)
        
    def state_callback(self, msg):
        # Activate node if mission state is idle
        if msg.mode == self.mode:
            self.is_active = True
        
    def AR_callback(self, msg):
        # offsets are between -0.5 and 0.5 and flipped to the FRD frame
        self.AR_y_offset = float(msg.x)/self.x_px - 0.5
        self.AR_x_offset = float(msg.y)/self.y_px - 0.5
        
        if msg.id == 0:
            self.AR_x_offset = 0.
            self.AR_y_offset = 0.

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        forward_velocity = self.forward_velocity_gain * self.AR_x_offset
        sideways_velocity = self.sideways_velocity_gain * self.AR_y_offset
        msg.velocity = {forward_velocity, sideways_velocity, 0.0}
        msg.yaw = 0.0
        self.publisher_.publish(msg)
    
    def publish_offboard_position_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    mode_hover = ModeHover()

    rclpy.spin(mode_hover)

    mode_hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()