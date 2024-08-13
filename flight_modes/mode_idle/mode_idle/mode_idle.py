import rclpy
from rclpy.node import Node

from px4_msgs import VehicleStatus
from tucan_msgs import Mode, ModeStatus

class ModeIdle(Node):
    """flight mode that waits for arming and setting to offboard mode
    """
    def __init__(self):
        super().__init__('mode_idle')
        self.get_logger().info('Idle mode initialized - waiting for arming')
        self.mode = 0 # Idle mode ID is 0
        self.__frequency = 1 # Frequency in Hz
        
        self.offboard_enabled = False
        self.armed = False
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(Mode, "mission_state", self.state_callback, 10)
        self.armed_subscriber_ = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.armed_callback, 10)
        
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "mode_status", 10)
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
    def state_callback(self, msg):
        # Activate node if mission state is idle
        if msg.mode == self.mode:
            self.is_active = True
        
    def armed_callback(self, msg):
        # If vehicle is armed, deactivate node and hand back control to mission director
        if msg.arming_state == msg.ARMING_STATE_ARMED:
            self.get_logger().info('Vehicle armed')
            self.armed = True
            self.publish_mode_status()
        else:
            self.get_logger().info('Vehicle not armed')
            self.armed = False
            self.publish_mode_status()
    
    def offboard_callback(self,msg):
        if msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info('Vehicle in offboard mode')
            self.offboard_enabled = True
            self.publish_mode_status()
        else:
            self.get_logger().info('Vehicle not in offboard mode')
            self.offboard_enabled = False
            self.publish_mode_status()
    
    def timer_callback(self):
        if self.armed and self.offboard_enabled:
            self.get_logger().info('Vehicle armed and in offboard mode - ready for takeoff')
            self.is_active = False
        else:
            self.get_logger().info('Vehicle disarmed - waiting for manual arming')
        self.publish_mode_status()
    
    def publish_mode_status(self):
        msg = ModeStatus()
        msg.mode = self.mode
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