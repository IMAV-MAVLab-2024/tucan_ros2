import rclpy
from rclpy.node import Node

from px4_msgs import TrajectorySetpoint
from tucan_msgs import ARMarker

class ModeHover(Node):
    """flight mode to hover above an AR marker
    """
    def __init__(self):
        super().__init__('mode_hover')
        self.get_logger().info('ModeHover initialized')
        self.__frequency = 20 # Frequency in Hz
        
        self.publisher_ = self.create_publisher(TrajectorySetpoint, "trajectory_setpoint", 10)
        
        self.AR_subsciber_ = self.create_subscription(ARMarker, "cv_ar_detection", self.AR_callback, 10)
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
        self.AR_x_offset = 0.
        self.AR_y_offset = 0.
        
        # Number of pixels in x and y direction of image
        self.x_px = 600.
        self.y_px = 400.
        
        # Gains for the velocity control
        self.forward_velocity_gain = 1.0
        self.sideways_velocity_gain = 1.0
            
    def timer_callback(self):
        self.publishTrajectorySetpoint()
        
    def AR_callback(self, msg):
        # offsets are between -0.5 and 0.5 and flipped to the FRD frame
        self.AR_y_offset = float(msg.x)/self.x_px - 0.5
        self.AR_x_offset = float(msg.y)/self.y_px - 0.5

    def publishTrajectorySetpoint(self):
        msg = TrajectorySetpoint()
        forward_velocity = self.forward_velocity_gain * self.AR_x_offset
        sideways_velocity = self.sideways_velocity_gain * self.AR_y_offset
        msg.velocity = {forward_velocity, sideways_velocity, 0.0}
        msg.yaw = 0.0
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    mode_hover = ModeHover()

    rclpy.spin(mode_hover)

    mode_hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()