import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8

import tucan_msgs.msg as tucan_msgs

import wiringpi, time

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

import sys

class DriverGripper(Node):
    """Gripper driver node
    """
    def __init__(self):
        super().__init__('driver_gripper')
        self.gripper_status_publisher = self.create_publisher(tucan_msgs.GripperStatus, 'gripper_status', 1)
        
        self.command_gripper = self.create_subscription(tucan_msgs.GripperCommand,"cmd_gripper", self.__listener_callback, 1, callback_group=ReentrantCallbackGroup())
        
        # settings
        self.frequency = 5

        self.status = tucan_msgs.GripperStatus()
        self.status.status = tucan_msgs.GripperStatus.OPENED

        self.timer = self.create_timer(1/self.frequency, self.__publish_status)

        # Clutch and continuous control settings
        self.us_clutch_engaged = 1600
        self.us_clutch_disengaged = 1750

        self.us_cont_rollup = 1028
        self.us_cont_rolloff = 1978
        self.us_cont_stop = 2978

        self.rollup_duration = 2.0 # 3.5
        self.engage_duration = 0.25

        wiringpi.wiringPiSetup()

        self.pin_cont = 5
        self.pin_clutch = 13

        wiringpi.pinMode(self.pin_cont, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetMode(self.pin_cont, wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pinMode(self.pin_clutch, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetMode(self.pin_clutch, wiringpi.GPIO.PWM_MODE_MS)


        # Set PWM range and clock
        self.pwm_range = 1000
        wiringpi.pwmSetRange(self.pin_cont, self.pwm_range)  # Set range (0-1023 for example)
        wiringpi.pwmSetRange(self.pin_clutch, self.pwm_range)  # Set range (0-1023 for example)
        self.servo_frequency = 50
        self.pwm_clock = 24000000
        pwmFrequency = int(self.pwm_clock / self.servo_frequency / self.pwm_range)
        wiringpi.pwmSetClock(self.pin_cont, pwmFrequency)   # Adjust clock for frequency control
        wiringpi.pwmSetClock(self.pin_clutch, pwmFrequency)   # Adjust clock for frequency control

        self.set_pwm_duty_cycle(self.pin_clutch, self.us_clutch_engaged)
        self.set_pwm_duty_cycle(self.pin_cont, self.us_cont_stop)

    
    def __publish_status(self):
        self.gripper_status_publisher.publish(self.status)

    def __listener_callback(self, msg):
        if msg.command == tucan_msgs.GripperCommand.OPEN:
            if self.status.status == tucan_msgs.GripperStatus.CLOSED:
                self.__gripper_start_open()
            else:
                self.get_logger().info('Gripper not in closed state, cannot open')
        elif msg.command == tucan_msgs.GripperCommand.CLOSE:
            if self.status.status == tucan_msgs.GripperStatus.OPENED:
                self.__gripper_start_close()
            else:
                self.get_logger().info('Gripper not in open state, cannot close')
    
    def __gripper_start_open(self):
        self.get_logger().info('Gripper opening')
        self.status.status = tucan_msgs.GripperStatus.OPENING
        self.set_pwm_duty_cycle(self.pin_clutch, self.us_clutch_engaged)
        self.set_pwm_duty_cycle(self.pin_cont, self.us_cont_rollup)
        self.__publish_status()

        time.sleep(self.rollup_duration)

        self.set_pwm_duty_cycle(self.pin_cont, self.us_cont_stop)
        self.status.status = tucan_msgs.GripperStatus.OPENED
        self.__publish_status()

    def __gripper_start_close(self):
        self.get_logger().info('Gripper closing')
        self.status.status = tucan_msgs.GripperStatus.CLOSING
        self.set_pwm_duty_cycle(self.pin_clutch, self.us_clutch_disengaged)
        self.set_pwm_duty_cycle(self.pin_cont, self.us_cont_rolloff)
        self.__publish_status()

        time.sleep(self.engage_duration)
        self.set_pwm_duty_cycle(self.pin_cont, self.us_cont_stop)
        self.status.status = tucan_msgs.GripperStatus.CLOSED
        self.__publish_status()

    def set_pwm_duty_cycle(self, pin, pulse_width_us):
        time_per_step = (1/self.servo_frequency * 10**6) / self.pwm_range  # Calculate time per step
        self.get_logger().debug(f'Time per step: {time_per_step}')
        duty_cycle_value = int(pulse_width_us / time_per_step)  # Convert us to PWM value
        self.get_logger().debug(f'Pulse width: {pulse_width_us}, Duty cycle: {duty_cycle_value}')
        wiringpi.pwmWrite(pin, duty_cycle_value)
    
def main(args=None):
    rclpy.init(args=args)
    gripper_driver_node = DriverGripper()
    executor = MultiThreadedExecutor()

    
    try:
        rclpy.spin(gripper_driver_node, executor=executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    
    gripper_driver_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()