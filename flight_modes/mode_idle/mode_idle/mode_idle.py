import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleStatus
from tucan_msgs.msg import Mode, ModeStatus

from rclpy.qos import QoSProfile, ReliabilityPolicy

class ModeIdle(Node):
    """flight mode that waits for arming and setting to offboard mode. Listens to the vehicle status topic and set itself to MODE_FINISHED when
    armed and in offboard mode.
    """
    def __init__(self):
        super().__init__('mode_idle')
        self.get_logger().info('Idle mode initialized')
        self.mode = 0 # Idle mode ID is 0
        self.__frequency = 1. # Frequency in Hz
        
        self.offboard_enabled = False
        self.armed = False
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(Mode, "/mission_state", self.state_callback, 10)
        QOSprofile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.armed_subscriber_ = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile=QOSprofile)
        
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "mode_status", 10)
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
    def state_callback(self, msg):
        # Activate node if mission state is idle
        if msg.mode_id == self.mode:
            self.is_active = True
        
    def vehicle_status_callback(self, msg):
        # If vehicle is armed, deactivate node and hand back control to mission director
        if msg.arming_state == msg.ARMING_STATE_ARMED:
            self.armed = True
        elif msg.arming_state != msg.ARMING_STATE_ARMED:
            self.armed = False # TO do make this false
        else:
            self.armed = False # To do make this false
                       
        if msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD:
            self.offboard_enabled = True
        elif msg.nav_state != msg.NAVIGATION_STATE_OFFBOARD:
            self.offboard_enabled = False
        else:
            self.offboard_enabled = False
    
    def timer_callback(self):
        if self.is_active:
            if self.armed and self.offboard_enabled:
                self.get_logger().info(f'Offboard {self.offboard_enabled} \t Armed {self.armed}')
                self.is_active = False
            elif self.offboard_enabled and not self.armed:
                self.get_logger().info(f'Offboard {self.offboard_enabled} \t Armed {self.armed}')
            elif not self.offboard_enabled and self.armed:
                self.get_logger().info(f'Offboard {self.offboard_enabled} \t Armed {self.armed}')
            else:
                self.get_logger().info(f'Offboard {self.offboard_enabled} \t Armed {self.armed}')
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
        
def main(args=None):
    rclpy.init(args=args)

    mode_idle = ModeIdle()

    rclpy.spin(mode_idle)

    mode_idle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()