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

class ModeWildlifePhotographer(Node):
    """flight mode to do wildlife photography task
    Orients the drone towards the screen, takes a bunch of pictures and then hands back control to the MD
    """
    def __init__(self):
        super().__init__('mode_wildlife_photographer')
        self.get_logger().info('Mode photography task initialized')
        self.mode = tucan_msgs.Mode.WILDLIFE_PHOTOGRAPHER 

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.is_active = False
        
        self.photo_taken = False # Start as false
        self.take_photo_counter = 0
        
        self.state_subscriber_ = self.create_subscription(tucan_msgs.Mode, "/active_mode_id", self.state_callback, 10)
        self.vehicle_odom_subscriber_ = self.create_subscription(px4_msgs.VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.alt_subscriber = self.create_subscription(std_msgs.Float32, "mode_wildlife_photographer/desired_altitude", self.desired_alt_callback, 5)
        self.id_subscriber = self.create_subscription(std_msgs.Int32, "mode_photographer/desired_id", self.desired_id_callback, 5)

        self.mode_status_publisher_ = self.create_publisher(tucan_msgs.ModeStatus, "/mode_status", 10)
        
        self.take_photo_publisher = self.create_publisher(std_msgs.Bool, "/take_photo", 10)
        
        self.setpoint_publisher_ = self.create_publisher(px4_msgs.TrajectorySetpoint, "/trajectory_setpoint", 10)
        self.control_mode_publisher = self.create_publisher(px4_msgs.OffboardControlMode, "/offboard_control_mode", 10)
        
        self.AR_subsciber_ = self.create_subscription(tucan_msgs.ARMarker, "/cv_aruco_detection", self.AR_callback, 10)
        
        self.ar_x = 0.
        self.ar_y = 0.
        self.ar_z = 0.

        self.yaw_tolerance = 0.1

        self.last_ar_time_tolerance = 3.5   # s, how long to use the last AR marker position after it has been lost
        
        # settings
        self.forward_pos_gain = 0.2
        self.sideways_pos_gain = 0.2
        
        self.start_yaw = 0.
        self.ar_yaw = 0. # rad
        self.desired_yaw = -math.pi/2 # rad
        self.desired_alt = 1.2 # m
        self.desired_ar_id = 105 # Set to 105 for competition

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
        if msg.mode_id == self.mode:
            if self.is_active == False:
                self.is_active = True
                self.publish_mode_status()
                self.get_logger().info(f'Photography mode started')

                self.start_yaw = self.quat_get_yaw(self.vehicle_odom_.q)
        else:
            self.is_active = False
        
    def AR_callback(self, msg):
        # Update the 
        if self.is_active:
            if not self.photo_taken:
                self.get_logger().info('No photo taken, aligning to wildlife')
                if msg.detected:
                    if self.desired_ar_id is None or self.desired_ar_id == msg.id:
                        # offsets are between -0.5 and 0.5 and flipped to the FRD frame
                        #self.get_logger().info(f'AR marker detected with ID {msg.id}')
                        #self.get_logger().info(f'Flying to x: {self.ar_x}, y: {self.ar_y}, z: {self.ar_z}')
                        self.ar_x = msg.x_global
                        self.ar_y = msg.y_global
                        self.ar_z = msg.z_global
                        self.ar_yaw = msg.yaw
                        self.publish_trajectory_setpoint()
                elif msg.id != 0: # ie an ar marker has been preiously found
                    time_difference = self.get_clock().now() - Time.from_msg(msg.last_detection_timestamp)

                    # Convert the result (which is an rclpy.duration.Duration object) to seconds
                    time_difference_seconds = time_difference.nanoseconds * 1e-9

                    #self.get_logger().info(f'no ar detection, age of old detection (s): {time_difference_seconds}')

                    if time_difference_seconds < self.last_ar_time_tolerance:
                        if self.desired_ar_id is None or self.desired_ar_id == msg.id:
                            #self.get_logger().info(f'No AR marker detected, flying to x: {self.ar_x}, y: {self.ar_y}, z: {self.ar_z}')
                            self.ar_x = msg.x_global
                            self.ar_y = msg.y_global
                            self.ar_z = msg.z_global
                            self.ar_yaw = msg.yaw
                            self.publish_trajectory_setpoint()
                
                # Check if yaw is achieved
                if self.ar_yaw + math.pi/2 < self.yaw_tolerance:
                    self.get_logger().info('Taking photos')
                    self.take_photo()
                    self.take_photo_counter+=1
                
                if self.take_photo_counter == 10:
                    self.photo_taken = True
                    self.get_logger().info("Photography mode completed")
                    self.is_active = False
                    
            self.publish_mode_status()
            self.publish_offboard_position_mode()

    def take_photo(self):
        photo_msg = std_msgs.Bool()
        photo_msg.data = True
        self.take_photo_publisher.publish(photo_msg)
        
    def vehicle_odom_callback(self, msg):
        self.vehicle_odom_ = msg

    def desired_alt_callback(self, msg):
        self.desired_alt = msg.data

    def publish_trajectory_setpoint(self):
        msg = px4_msgs.TrajectorySetpoint()

        x_desired = self.ar_x
        y_desired = self.ar_y
        z_desired = self.ar_z - self.desired_alt

        if self.ar_yaw is None or self.desired_yaw is None:
            desired_yaw = self.start_yaw
        else:
            current_yaw = self.quat_get_yaw(self.vehicle_odom_.q)
            desired_yaw = current_yaw + self.ar_yaw + self.desired_yaw # orient yourself to pi/2 w.r.t. the marker

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
    
    def desired_id_callback(self, msg):
        self.desired_ar_id = msg.data

def main(args=None):
    rclpy.init(args=args)

    mode_photographer = ModeWildlifePhotographer()

    rclpy.spin(mode_photographer)

    mode_photographer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()