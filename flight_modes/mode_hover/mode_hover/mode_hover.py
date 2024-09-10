import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import tucan_msgs.msg as tucan_msgs
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
import tucan_msgs.msg as tucan_msgs

class ModeHover(Node):
    """flight mode to hover above an AR marker.
    Runs a timer that publishes the mode status every 1/frequency seconds and if node is set to active, publishes velocity setpoints.
    Has a small internal control loop that adjusts the velocity setpoints based on the AR marker position. If no marker is detected, keeps position.
    """
    def __init__(self):
        super().__init__('mode_hover')
        self.get_logger().info('ModeHover initialized')
        self.mode = tucan_msgs.Mode.HOVER # Hover mode ID is 1, DON'T CHANGE
        self.__frequency = 10 # Frequency in Hz

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(tucan_msgs.Mode, "/active_mode_id", self.state_callback, 10)
        self.vehicle_odom_subscriber_ = self.create_subscription(px4_msgs.VehicleOdometry, "fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)

        self.mode_status_publisher_ = self.create_publisher(tucan_msgs.ModeStatus, "/mode_status", 10)
        
        self.setpoint_publisher_ = self.create_publisher(px4_msgs.TrajectorySetpoint, "/trajectory_setpoint", 10)
        self.control_mode_publisher = self.create_publisher(px4_msgs.OffboardControlMode, "/offboard_control_mode", 10)
        
        self.AR_subsciber_ = self.create_subscription(tucan_msgs.ARMarker, "/cv_aruco_detection", self.AR_callback, 10)
        
        self.AR_x_offset = 0.
        self.AR_y_offset = 0.
        
        # settings
        self.forward_pos_gain = 0.2
        self.sideways_pos_gain = 0.2
        self.x_px = 800
        self.y_px = 600

        self.vehicle_odom_position_ = None
        
    def publish_mode_status(self):
        msg = tucan_msgs.ModeStatus()
        msg.mode.mode_id = self.mode
        if self.is_active:
            msg.mode_status = msg.MODE_ACTIVE
        else:
            msg.mode_status = msg.MODE_FINISHED

        msg.busy = False
        self.mode_status_publisher_.publish(msg)
        
    def state_callback(self, msg):
        # Activate node if mission state is idle
        if msg.mode_id == self.mode:
            self.is_active = True
            self.publish_mode_status()
        else:
            self.is_active = False
        
    def AR_callback(self, msg):
        # Update the 
        if self.is_active:
            if msg.detected:
                # offsets are between -0.5 and 0.5 and flipped to the FRD frame
                self.AR_y_offset = float(msg.x)/self.x_px - 0.5
                self.AR_x_offset = float(msg.y)/self.y_px - 0.5
                
                if msg.id == 0:
                    self.AR_x_offset = 0.
                    self.AR_y_offset = 0.
            
                self.publish_trajectory_setpoint()
                
            self.publish_mode_status()
            self.publish_offboard_position_mode()

    def vehicle_odom_callback(self, msg):
        self.vehicle_odom_ = msg

    def publish_trajectory_setpoint(self):
        msg = px4_msgs.TrajectorySetpoint()
        forward_pos = self.vehicle_odom_.position[0] + self.forward_pos_gain * self.AR_x_offset
        sideways_pos = self.vehicle_odom_.position[1] + self.sideways_pos_gain * self.AR_y_offset
        msg.position = [float(forward_pos), float(sideways_pos), float(1.2)]
        self.get_logger().info(f'Forward offset {self.forward_pos_gain * self.AR_x_offset}; Sideways offset {self.sideways_pos_gain * self.AR_y_offset}')
        msg.yaw = 0.0
        self.setpoint_publisher_.publish(msg)
    
    def publish_offboard_position_mode(self):
        msg = px4_msgs.OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    mode_hover = ModeHover()

    rclpy.spin(mode_hover)

    mode_hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()