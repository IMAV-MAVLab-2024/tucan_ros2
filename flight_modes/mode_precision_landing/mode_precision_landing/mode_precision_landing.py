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

from rclpy.time import Time
from builtin_interfaces.msg import Time as HeaderTime

import std_msgs.msg as std_msgs

class ModePrecisionLanding(Node):
    """Flight mode for landing on an AR marker
    """
    def __init__(self):
        super().__init__('mode_hover')
        self.get_logger().info('ModeHover initialized')
        self.mode = tucan_msgs.Mode.PRECISION_LANDING
        self.__frequency = 10 # Frequency in Hz

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(tucan_msgs.Mode, "/active_mode_id", self.state_callback, 10)
        self.vehicle_odom_subscriber_ = self.create_subscription(px4_msgs.VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.yaw_subscriber = self.create_subscription(std_msgs.Float32, "mode_precision_landing/desired_yaw", self.desired_yaw_callback, 5)
        self.id_subscriber = self.create_subscription(std_msgs.Int32, "mode_precision_landing/desired_id", self.desired_id_callback, 5)

        self.mode_status_publisher_ = self.create_publisher(tucan_msgs.ModeStatus, "/mode_status", 10)
        
        self.setpoint_publisher_ = self.create_publisher(px4_msgs.TrajectorySetpoint, "/trajectory_setpoint", 10)
        self.control_mode_publisher = self.create_publisher(px4_msgs.OffboardControlMode, "/offboard_control_mode", 10)
        
        self.AR_subsciber_ = self.create_subscription(tucan_msgs.ARMarker, "/cv_aruco_detection", self.AR_callback, 10)
        
        self.ar_x = None
        self.ar_y = None
        self.ar_z = None

        self.initial_x = None
        self.initial_y = None
        self.initial_z = None

        self.landing_started = False
        self.landing_tolerance = 0.2 # m, how close to the marker horizontally before we have to start descent
        self.landing_tolerance_sq = self.landing_tolerance  # because square roots are slow

        self.last_ar_time_tolerance = 3.5   # s, how long to use the last AR marker position after it has been lost
        
        # settings
        self.landing_speed_gain = 0.065 # m, how far the position setpoint should be from the current position of the drone, translates proportionally to landing speed

        self.desired_yaw = 0.0 # rad
        self.desired_ar_id = None

        self.vehicle_odom_ = None
        
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
            if self.is_active == False:
                self.is_active = True
                self.landing_started = False
                self.publish_mode_status()
                self.get_logger().info(f'Landing mode started')

                if self.desired_yaw is None:
                    self.desired_yaw = self.quat_get_yaw(self.vehicle_odom_.q)

                self.initial_x = self.vehicle_odom_.position[0]
                self.initial_y = self.vehicle_odom_.position[1]
                self.initial_z = self.vehicle_odom_.position[2]
        else:
            if self.is_active:
                self.desired_yaw = None
            self.is_active = False
        
    def AR_callback(self, msg):
        # Update the 
        if self.is_active:
            trajectory_set = False

            if msg.detected:
                if self.desired_ar_id is None or self.desired_ar_id == msg.id:
                    # offsets are between -0.5 and 0.5 and flipped to the FRD frame
                    self.get_logger().info(f'AR marker detected with ID {msg.id}')
                    self.get_logger().info(f'Flying to x: {self.ar_x}, y: {self.ar_y}, z: {self.ar_z}')
                    self.ar_x = msg.x_global
                    self.ar_y = msg.y_global
                    self.ar_z = msg.z_global
                    trajectory_set = True
                    self.publish_trajectory_setpoint()
            elif msg.id != 0: # ie an ar marker has been preiously found
                time_difference = self.get_clock().now() - Time.from_msg(msg.last_detection_timestamp)

                # Convert the result (which is an rclpy.duration.Duration object) to seconds
                time_difference_seconds = time_difference.nanoseconds * 1e-9

                self.get_logger().info(f'no ar detection, age of old detection (s): {time_difference_seconds}')

                if time_difference_seconds < self.last_ar_time_tolerance:
                    if self.desired_ar_id is None or self.desired_ar_id == msg.id:
                        self.get_logger().info(f'No AR marker detected, flying to x: {self.ar_x}, y: {self.ar_y}, z: {self.ar_z}')
                        self.ar_x = msg.x_global
                        self.ar_y = msg.y_global
                        self.ar_z = msg.z_global

                        trajectory_set = True
                        self.publish_trajectory_setpoint()
            
            if self.landing_started and not trajectory_set:
                self.publish_trajectory_setpoint()
                    
            self.publish_mode_status()
            self.publish_offboard_position_mode()

    def vehicle_odom_callback(self, msg):
        self.vehicle_odom_ = msg

    def desired_yaw_callback(self, msg):
        self.desired_yaw = msg.data

    def desired_alt_callback(self, msg):
        self.desired_alt = msg.data

    def desired_id_callback(self, msg):
        self.desired_id = msg.data

    def publish_trajectory_setpoint(self):
        msg = px4_msgs.TrajectorySetpoint()

        # are we within tolerancecm of the marker in xy? if so start landing

        if not self.landing_started:
            distance = (self.ar_x - self.vehicle_odom_.position[0]) ** 2 + (self.ar_y - self.vehicle_odom_.position[1]) ** 2
            if distance < self.landing_tolerance_sq:
                self.landing_started = True
                self.get_logger().info('ar marker reached, Starting landing')

        if self.landing_started:
            x_desired = self.ar_x
            y_desired = self.ar_y
            z_desired = self.vehicle_odom_.position[1] + self.landing_speed_gain
        else:
            x_desired = self.ar_x
            y_desired = self.ar_y
            z_desired = self.initial_z

        msg.position = [float(x_desired), float(y_desired), float(z_desired)]
        self.get_logger().info(f'x_des: {x_desired}, y_des: {y_desired}, z_des: {z_desired}')
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

    mode_hover = ModePrecisionLanding()

    rclpy.spin(mode_hover)

    mode_hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()