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

class ModeVerticalGate(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_vertical_gate')
        self.mode = Mode.VERTICAL_GATE # Gate mode ID is 4, DON'T CHANGE
        self.declare_parameter('which_gate', 'middle')
        self.gate = self.get_parameter('which_gate').get_parameter_value().string_value
        self.state_subscriber = self.create_subscription(Mode,'/active_mode_id', self.__listener_callback,1)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.vehicle_odom_subscriber_ = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "/mode_status", 10)
        self.is_active = False
        
        self.control_mode_publisher_ = self.create_publisher(Mode, "/control_mode", 10)
        self.setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/trajectory_setpoint", 10)
        
        self.odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.__odometry_callback, 1)
        self.position = [0.0, 0.0, 0.0]
        
        self.ar_subscriber = self.create_subscription(ARMarker, "/cv_aruco_detection", self.AR_callback, 1)
        self.last_ar_marker = 0
        
        self.frequency = 1. # Node frequency in Hz
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
        self.aligned = False
        
        self.yaw_tolerance = 0.1 # Tolerance in radians for the yaw alignment
        
        if self.gate == 'top':
            self.target_altitude = 2.1225 # Altitude of the top gate in meters
        if self.gate == 'middle':
            self.target_altitude = 1.75 # Altitude of the middle gate in meters
        if self.gate == 'bottom':
            self.target_altitude = 1.0 # Altitude of the bottom gate in meters
        
    def execute(self):
        """Execute the vertical gate mode.
        """
        if not self.aligned:
        # Align drone with the AR marker (relative angle should be pi/2)
            self.desired_yaw = self.current_yaw - self.ar_yaw + math.pi/2.
            self.desired_x = self.vehicle_odom_.position[0]
            self.publish_trajectory_setpoint()
            if self.ar_yaw-math.pi/2. < self.yaw_tolerance:
                self.aligned = True
        
        elif self.aligned:
            # Fly forward until the marker
            self.desired_x = self.vehicle_odom_.position[0] + 0.5
            
            if self.last_ar_marker == 205: # after gate marker detected
                self.is_active = False # Deactivate node
        
    def __listener_callback(self, msg):
        if msg.mode_id == self.mode:
            self.is_active = True

    def timer_callback(self):
        if self.is_active:
            self.publish_mode_status()
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
        msg.position = {self.desired_x, self.vehicle_odom_.position[1], self.target_altitude}
        msg.yaw = self.desired_yaw
        self.setpoint_publisher.publish(msg)
    
    def publish_offboard_position_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher.publish(msg)
    
    def __odometry_callback(self, msg):
        self.position = msg.position
    
    def AR_callback(self,msg):
        self.last_ar_marker = msg.id
        self.ar_yaw = msg.yaw
        

    def vehicle_odom_callback(self, msg):
        self.vehicle_odom_ = msg
        
        
def main(args=None):
    rclpy.init(args=args)
    mode_vertical_gate = ModeVerticalGate()

    rclpy.spin(mode_vertical_gate)

    mode_vertical_gate.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
