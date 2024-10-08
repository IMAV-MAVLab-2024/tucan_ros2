import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy

import time
import math

from tucan_msgs.msg import Mode, ModeStatus, ARMarker
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry

import std_msgs.msg as std_msgs

class ModeVerticalGate(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_vertical_gate')
        self.mode = Mode.VERTICAL_GATE # Gate mode ID is 4, DON'T CHANGE

        # parameters
        self.declare_parameter('which_gate', 'middle')
        self.gate = self.get_parameter('which_gate').get_parameter_value().string_value
        
        # subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.state_subscriber = self.create_subscription(Mode,'/active_mode_id', self.__listener_callback,1)
        self.vehicle_odom_subscriber_ = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.ar_subscriber = self.create_subscription(ARMarker, "/cv_aruco_detection", self.AR_callback, 1)
        self.desired_ar_id_subscriber = self.create_subscription(std_msgs.Int32, "/mode_gate/desired_id", self.process_desired_ar_id, 1)
        self.odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.__odometry_callback, 1)        

        # publishers
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "/mode_status", 10)
        self.control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/control_mode", 10)
        self.setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/trajectory_setpoint", 10)   
        
        self.position = [0.0, 0.0, 0.0]
        self.is_active = False
        self.desired_ar_marker_id = 205
        self.last_ar_marker = 0
        
        self.frequency = 10. # Node frequency in Hz
        self.timer = self.create_timer(1./self.frequency, self.timer_callback)
        
        if self.gate == 'top':
            self.target_altitude = 2.1225 # Altitude of the top gate in meters
        if self.gate == 'middle':
            self.target_altitude = 1.75 # Altitude of the middle gate in meters
        if self.gate == 'bottom':
            self.target_altitude = 1.0 # Altitude of the bottom gate in meters
        
        self.forward_setpoint = 0.2
        
    def execute(self):
        """Execute the vertical gate mode.
        """
        # Align drone with the AR marker (relative angle should be pi/2)
        self.yaw = self.start_yaw # Actual yaw is desired yaw for flying forward

        self.desired_x = self.vehicle_odom_.position[0] + (math.cos(self.start_yaw)*self.forward_setpoint)
        self.desired_y = self.vehicle_odom_.position[1] + (math.sin(self.start_yaw)*self.forward_setpoint)

        self.publish_trajectory_setpoint()

    def __listener_callback(self, msg):
        if msg.mode_id == self.mode:
            if self.is_active == False:
                self.finished = False
                self.start_yaw = self.quat_get_yaw(self.vehicle_odom_.q)
            self.is_active = True
        else:
            self.is_active = False

    def timer_callback(self):
        if self.is_active:
            self.publish_mode_status()
            if not self.finished:
                self.execute()

    def publish_mode_status(self):
        msg = ModeStatus()
        msg.mode.mode_id = self.mode
        if self.is_active:
            if self.finished:
                msg.mode_status = msg.MODE_FINISHED
            else:
                msg.mode_status = msg.MODE_ACTIVE
        else:
            msg.mode_status = msg.MODE_INACTIVE
        msg.busy = self.is_active
        self.mode_status_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.desired_x), float(self.desired_y), float(-self.target_altitude)]
        msg.yaw = self.yaw
        self.setpoint_publisher_.publish(msg)
    
    def publish_offboard_position_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher_.publish(msg)
    
    def __odometry_callback(self, msg):
        self.position = msg.position
    
    def AR_callback(self,msg):
        self.last_ar_marker = msg.id
        self.ar_yaw = msg.yaw
        if self.last_ar_marker == self.desired_ar_marker_id:
            self.finished = True

    def vehicle_odom_callback(self, msg):
        self.vehicle_odom_ = msg
    
    def process_desired_ar_id(self,msg):
        self.desired_ar_marker_id = msg.data
        
    def quat_get_yaw(self, q):
        q_w = q[0]
        q_x = q[1]
        q_y = q[2]
        q_z = q[3]
        return math.atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))
    
def main(args=None):
    rclpy.init(args=args)
    mode_vertical_gate = ModeVerticalGate()

    rclpy.spin(mode_vertical_gate)

    mode_vertical_gate.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
