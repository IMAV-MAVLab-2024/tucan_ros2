import rclpy
from rclpy.node import Node

import math

import px4_msgs.msg as px4_msgs
import tucan_msgs.msg as tucan_msgs
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
import tucan_msgs.msg as tucan_msgs

import std_msgs.msg as std_msgs

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
        self.yaw_subscriber = self.create_subscription(std_msgs.Float32, "mode_hover/desired_yaw", self.desired_yaw_callback, 5)
        self.alt_subscriber = self.create_subscription(std_msgs.Float32, "mode_hover/desired_altitude", self.desired_alt_callback, 5)

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

        self.desired_yaw = 0.0 # rad
        self.desired_alt = 1.0 # m

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

    def desired_yaw_callback(self, msg):
        self.desired_yaw = msg.data

    def desired_alt_callback(self, msg):
        self.desired_alt = msg.data

    def publish_trajectory_setpoint(self):
        msg = px4_msgs.TrajectorySetpoint()

        current_yaw = self.quat_get_yaw(self.vehicle_odom_.q)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        forward_pos = self.vehicle_odom_.position[0] + self.forward_pos_gain * self.AR_x_offset * cos_yaw - self.sideways_pos_gain * self.AR_y_offset * sin_yaw
        sideways_pos = self.vehicle_odom_.position[1] + self.forward_pos_gain * self.AR_x_offset * sin_yaw + self.sideways_pos_gain * self.AR_y_offset * cos_yaw

        msg.position = [float(forward_pos), float(sideways_pos), float(self.desired_alt)]
        self.get_logger().info(f'Forward offset {self.forward_pos_gain * self.AR_x_offset}; Sideways offset {self.sideways_pos_gain * self.AR_y_offset}')
        msg.yaw = self.desired_yaw
        self.setpoint_publisher_.publish(msg)
    
    def publish_offboard_position_mode(self):
        msg = px4_msgs.OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher.publish(msg)

    def quat_get_yaw(self, q):
        q_w = q[0]
        q_x = q[1]
        q_y = q[2]
        q_z = q[3]
        return math.atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))
        
def main(args=None):
    rclpy.init(args=args)

    mode_hover = ModeHover()

    rclpy.spin(mode_hover)

    mode_hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()