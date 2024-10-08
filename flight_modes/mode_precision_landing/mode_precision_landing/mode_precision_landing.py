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

import time

class ModePrecisionLanding(Node):
    """Flight mode for landing on an AR marker
    """
    def __init__(self):
        super().__init__('mode_precision_landing')
        self.get_logger().info('Mode precision landing initialized')
        self.mode = tucan_msgs.Mode.PRECISION_LANDING

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.armed_subscriber_ = self.create_subscription(px4_msgs.VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile=qos_profile)

        
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(tucan_msgs.Mode, "/active_mode_id", self.state_callback, 10)
        self.vehicle_odom_subscriber_ = self.create_subscription(px4_msgs.VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.yaw_subscriber = self.create_subscription(std_msgs.Float32, "mode_precision_landing/desired_relative_yaw", self.desired_yaw_callback, 5)
        self.id_subscriber = self.create_subscription(std_msgs.Int32, "mode_precision_landing/desired_id", self.desired_id_callback, 5)

        self.mode_status_publisher_ = self.create_publisher(tucan_msgs.ModeStatus, "/mode_status", 10)
        
        self.vehicle_command_publisher_ = self.create_publisher(px4_msgs.VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.setpoint_publisher_ = self.create_publisher(px4_msgs.TrajectorySetpoint, "/trajectory_setpoint", 10)
        self.control_mode_publisher = self.create_publisher(px4_msgs.OffboardControlMode, "/offboard_control_mode", 10)
        
        self.AR_subsciber_ = self.create_subscription(tucan_msgs.ARMarker, "/cv_aruco_detection", self.AR_callback, 10)
        
        self.ar_x = None
        self.ar_y = None
        self.ar_z = None
        self.ar_yaw = None

        self.initial_x = None
        self.initial_y = None
        self.initial_z = None

        self.start_yaw = None

        self.maybe_landed = False
        self.maybe_landed_start_time = None
        self.landing_threshhold = 0.15      # m
        self.landing_time = 1.0             # s
        self.disarming = False

        self.landing_started = False
        self.landing_finished = False
        self.landing_tolerance = 0.4 # m, how close to the marker horizontally before we have to start descent
        self.landing_tolerance_sq = self.landing_tolerance  # because square roots are slow

        self.last_ar_time_tolerance = 3.5   # s, how long to use the last AR marker position after it has been lost
        
        # settings
        self.landing_speed_gain = 0.6 # m, how far the position setpoint should be from the current position of the drone, translates proportionally to landing speed

        self.desired_yaw = None # rad
        self.desired_ar_id = None

        self.vehicle_odom_ = None

        self.mode_status_frequency = 5
        self.mode_status_timer = self.create_timer(1./self.mode_status_frequency, self.publish_mode_status)
        
    def publish_mode_status(self):
        if self.is_active:
            msg = tucan_msgs.ModeStatus()
            msg.mode.mode_id = self.mode
            if self.landing_finished:
                msg.mode_status = msg.MODE_FINISHED
            else:
                msg.mode_status = msg.MODE_ACTIVE

            msg.busy = False
            self.mode_status_publisher_.publish(msg)
        
    def state_callback(self, msg):
        # Activate node if mission state is idle
        if msg.mode_id == self.mode:
            if self.is_active == False:
                self.is_active = True
                self.landing_started = False
                self.landing_finished = False
                self.get_logger().info(f'Landing mode started')

                self.start_yaw = self.quat_get_yaw(self.vehicle_odom_.q)

                self.initial_x = self.vehicle_odom_.position[0]
                self.initial_y = self.vehicle_odom_.position[1]
                self.initial_z = self.vehicle_odom_.position[2]

                self.publish_mode_status()
        else:
            if self.is_active:
                self.desired_yaw = None
                self.ar_yaw = None
                self.maybe_landed = False
                self.disarming = False
            self.is_active = False
        
    def AR_callback(self, msg):
        # Update the 
        if self.is_active and not self.landing_finished:
            trajectory_set = False

            if msg.detected:
                if self.desired_ar_id is None or self.desired_ar_id == msg.id:
                    # offsets are between -0.5 and 0.5 and flipped to the FRD frame
                    self.get_logger().debug(f'AR marker detected with ID {msg.id}')
                    self.get_logger().debug(f'Flying to x: {self.ar_x}, y: {self.ar_y}, z: {self.ar_z}')
                    
                    self.ar_x = msg.x_global
                    self.ar_y = msg.y_global
                    self.ar_z = msg.z_global
                    self.ar_yaw = msg.yaw

                    trajectory_set = True
                    self.publish_trajectory_setpoint()
            elif msg.id != 0: # ie an ar marker has been preiously found
                time_difference = self.get_clock().now() - Time.from_msg(msg.last_detection_timestamp)

                # Convert the result (which is an rclpy.duration.Duration object) to seconds
                time_difference_seconds = time_difference.nanoseconds * 1e-9

                self.get_logger().debug(f'no ar detection, age of old detection (s): {time_difference_seconds}')

                if time_difference_seconds < self.last_ar_time_tolerance:
                    if self.desired_ar_id is None or self.desired_ar_id == msg.id:
                        self.get_logger().debug(f'No AR marker detected, flying to x: {self.ar_x}, y: {self.ar_y}, z: {self.ar_z}')
                        
                        self.ar_x = msg.x_global
                        self.ar_y = msg.y_global
                        self.ar_z = msg.z_global
                        self.ar_yaw = msg.yaw

                        trajectory_set = True
                        self.publish_trajectory_setpoint()
            
            if self.landing_started and not trajectory_set:
                self.publish_trajectory_setpoint()
                    
            self.publish_mode_status()
            self.publish_offboard_position_mode()

    def vehicle_odom_callback(self, msg):
        self.vehicle_odom_ = msg
        
        if self.is_active and self.landing_started:
            self.publish_trajectory_setpoint() 

            if not self.landing_finished and not self.disarming:
                if self.maybe_landed:
                    if -msg.position[2] > self.landing_tolerance:
                        self.maybe_landed = False
                        self.get_logger().info('maybe landed reset')
                    elif (time.time() - self.maybe_landed_start_time) > self.landing_time:
                        self.disarming = True
                else:
                    if -msg.position[2] < self.landing_tolerance:
                        self.maybe_landed = True
                        self.maybe_landed_start_time = time.time()

                        self.get_logger().info('maybe landed set')
            if self.disarming:  
                msg_disarm = px4_msgs.VehicleCommand()  
                    
                msg_disarm.command = px4_msgs.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                msg_disarm.param1 = float(px4_msgs.VehicleCommand.ARMING_ACTION_DISARM)
                msg_disarm.param2 = float(21196) #force disarm

                msg_disarm.target_system = 1
                msg_disarm.target_component = 1
                msg_disarm.source_system = 1
                msg_disarm.source_component = 1
                msg_disarm.from_external = True
                msg_disarm.timestamp = int(self.get_clock().now().nanoseconds / 1000)

                self.vehicle_command_publisher_.publish(msg_disarm)

    def vehicle_status_callback(self, msg):    
        if self.landing_started and msg.arming_state == msg.ARMING_STATE_DISARMED:
            self.landing_finished = True

    def desired_yaw_callback(self, msg):
        self.desired_yaw = msg.data

    def desired_alt_callback(self, msg):
        self.desired_alt = msg.data

    def desired_id_callback(self, msg):
        self.desired_ar_id = msg.data

    def publish_trajectory_setpoint(self):
        if not self.landing_finished:
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
                z_desired = self.vehicle_odom_.position[2] + self.landing_speed_gain
            else:
                x_desired = self.ar_x
                y_desired = self.ar_y
                z_desired = self.initial_z

            if self.ar_yaw is None or self.desired_yaw is None:
                desired_yaw = self.start_yaw
            else:
                desired_yaw = self.ar_yaw + self.desired_yaw # orient yourself to pi/2 w.r.t. the marker


            msg.position = [float(x_desired), float(y_desired), float(z_desired)]
            #self.get_logger().info(f'x_des: {x_desired}, y_des: {y_desired}, z_des: {z_desired}')
            msg.yaw = desired_yaw
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

    mode_land = ModePrecisionLanding()

    rclpy.spin(mode_land)

    mode_land.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()