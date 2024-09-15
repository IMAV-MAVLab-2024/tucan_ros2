import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
from tucan_msgs.msg import Mode, ModeStatus

from rclpy.qos import QoSProfile, ReliabilityPolicy

class ModeArm(Node):
    """flight mode that arms and the drone, use with caution
    """
    def __init__(self):
        super().__init__('mode_arm')
        self.get_logger().info('Arm mode initialized')
        self.mode = Mode.ARM # Idle mode ID is 0
        self.__frequency = 5. # Frequency in Hz
        
        self.offboard_enabled = False
        self.armed = False
        self.is_active = False
        
        self.state_subscriber_ = self.create_subscription(Mode, "/active_mode_id", self.state_callback, 10)
        QOSprofile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.armed_subscriber_ = self.create_subscription(px4_msgs.VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile=QOSprofile)

        self.vehicle_command_publisher_ = self.create_publisher(px4_msgs.VehicleCommand, "/fmu/in/vehicle_command", 10)
        
        self.mode_status_publisher_ = self.create_publisher(ModeStatus, "mode_status", 10)
        
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)
        
    def state_callback(self, msg):
        # Activate node if mission state is idle
        if msg.mode_id == self.mode:
            if not self.is_active:
                self.get_logger().info("Arming mode started")
            self.is_active = True
        else:
            self.is_active = False
        
    def vehicle_status_callback(self, msg):
        if msg.arming_state == msg.ARMING_STATE_ARMED:
            self.armed = True
        else:
            self.armed = False

        if msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD:
            self.offboard_enabled = True
        else:
            self.offboard_enabled = False
    
    def timer_callback(self):
        if self.is_active:
            if self.armed:
                if self.offboard_enabled:
                    self.is_active = False
                else:
                    self.get_logger().info("Waiting for offboard mode to be enabled")
            else:
                if self.offboard_enabled:
                    self.arm()
                else:
                    self.get_logger().info("Waiting for offboard mode to be enabled")

            self.publish_mode_status()

    def arm(self):
        msg_arm = px4_msgs.VehicleCommand()
        msg_arm.command = px4_msgs.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg_arm.param1 = float(px4_msgs.VehicleCommand.ARMING_ACTION_ARM)

        msg_arm.target_system = 1
        msg_arm.target_component = 1
        msg_arm.source_system = 1
        msg_arm.source_component = 1
        msg_arm.from_external = True
        msg_arm.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_publisher_.publish(msg_arm)
    
    def publish_mode_status(self):
        msg = ModeStatus()
        msg.mode.mode_id = self.mode
        if self.is_active:
            msg.mode_status = msg.MODE_ACTIVE
        else:
            msg.mode_status = msg.MODE_FINISHED
        msg.busy = False
        self.mode_status_publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    mode_arm = ModeArm()

    rclpy.spin(mode_arm)

    mode_arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()