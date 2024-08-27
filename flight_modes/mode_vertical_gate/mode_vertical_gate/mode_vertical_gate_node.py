import rclpy
from rclpy.node import Node

import time

from tucan_msgs.msg import Mode, ModeStatus, ARMarker
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry

class ModeVerticalGate(Node):
    """Sample placing mode node.
    """
    def __init__(self):
        super().__init__('mode_vertical_gate')
        self.mode = 4 # Gate mode ID is 4, DON'T CHANGE
        self.declare_parameter('which_gate', 'top')
        self.gate = self.get_parameter('which_gate').get_parameter_value().string_value
        self.state_subscriber = self.create_subscription(Mode,'/mission_state', self.__listener_callback,1)
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "/mode_status", 10)
        self.is_active = False
        
        self.control_mode_publisher_ = self.create_publisher(Mode, "/control_mode", 10)
        self.setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/trajectory_setpoint", 10)
        
        self.odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.__odometry_callback, 1)
        self.position = [0.0, 0.0, 0.0]
        
        self.ar_subscriber = self.create_subscription(ARMarker, "/cv_aruco_detection", self.__ar_callback, 1)
        self.last_ar_marker = 0
        
        self.frequency = 1. # Node frequency in Hz
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
        if self.gate == 'top':
            self.target_altitude = 2.1225 # Altitude of the top gate in meters
        if self.gate == 'middle':
            self.target_altitude = 1.75 # Altitude of the middle gate in meters
        if self.gate == 'bottom':
            self.target_altitude = 1.0 # Altitude of the bottom gate in meters
        
        
    def execute(self):
        """Execute the vertical gate mode.
        """
        # First publish a setpoint for the target altitude
        msg = TrajectorySetpoint()
        msg.position = {self.position[0], self.position[1], self.target_altitude}
        msg.yaw = 0.0
        self.setpoint_publisher.publish(msg)
        
        # Wait for the drone to get to altitude
        while abs(self.target_altitude-self.position[3])< 0.1:
            time.sleep(0.01)

        # Change to velocity mode
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher.publish(msg)
        
        # Publish forward velocity
        msg = TrajectorySetpoint()
        msg.velocity = {0.2, 0.0, 0.0}
        msg.yaw = 0.0
        self.setpoint_publisher.publish(msg)
        
        # Wait until AR marker is found
        while (self.last_ar_marker!=100): # TO DO: set to right id
            time.sleep(0.01)
        
        # Deactivate node to go to next state
        self.is_active = False
        self.publish_mode_status()
        
    def __listener_callback(self, msg):
        if msg.mode_id == self.mode:
            self.is_active = True
            self.execute()
                
    def timer_callback(self):
        if self.is_active:
            self.publish_mode_status()
        
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
        msg.position = {0., 0., self.target_altitude}
        msg.yaw = 0.0
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
    
    def __ar_callback(self,msg):
        self.last_ar_marker = msg.id
        
        
def main(args=None):
    rclpy.init(args=args)
    mode_vertical_gate = ModeVerticalGate()

    rclpy.spin(mode_vertical_gate)

    mode_vertical_gate.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
