import rospy
from gripper_control.msg import servo_feedback
from gripper_control.msg import servo_speed
from gripper_control.msg import servo_position
from gripper_control.msg import servo_pwm
from gripper_control.msg import servo_enable_torque
from gripper_control.msg import servo_calibrate_zero
from gripper_control.msg import gripper_cmd
import scripts.teensy_python_interface as teensy
import time
import json
import std_msgs.msg
import threading


# gripper command ID
GRIPPER_MORPHING_X = 1
GRIPPER_MORPHING_BX = 2
GRIPPER_MORPHING_BY = 3
GRIPPER_TRIGGER_ON = 4
GRIPPER_TRIGGER_OFF = 5
GRIPPER_OPEN = 6
GRIPPER_CLOSE_SLOW = 7
GRIPPER_CLOSE_FAST = 8

GRIPPER_FREE = 66
GRIPPER_ZERO = 88

class gripperLM:
    def __init__(self):

        rospy.init_node('gripper_control_node', anonymous=True)

        #get parameters from launch file

        self.servo_list = [3]
        self.feedback_frequency = float(rospy.get_param('~feedback_frequency', 3.0))


        self.pub_servo_pwm = rospy.Publisher('/gripper/servo_pwm', servo_pwm, queue_size=10)
        self.pub_feetech_speed = rospy.Publisher('/gripper/servo_set_speed', servo_speed, queue_size=10)
        self.pub_feetech_pos = rospy.Publisher('/gripper/servo_set_position', servo_position, queue_size=10)
        self.pub_feetech_enable = rospy.Publisher('/gripper/servo_enable_torque', servo_enable_torque, queue_size=10)
        self.pub_feetech_zero = rospy.Publisher('/gripper/servo_set_zero_position', servo_calibrate_zero, queue_size=10)
        
        rospy.Subscriber("/gripper/command", gripper_cmd , self.callback_gripper_cmd,queue_size=1)

        # Feetech servo setup
        self.FeetechID = 3
        self.FeetechSpeed = servo_speed()
        self.FeetechSpeed.servo_id = self.FeetechID
        self.FeetechSpeedSlow = 1
        self.FeetechSpeedFast = 100

        self.FeetechPosition = servo_position()
        self.FeetechPosition.servo_id = self.FeetechID
        self.FeetechPositionOpen = 4089
        self.FeetechPositionClose = 100

        self.FeetechEnableTorque = servo_enable_torque()
        self.FeetechEnableTorque.servo_id = self.FeetechID

        self.FeetechCalibrateZero = servo_calibrate_zero()
        self.FeetechCalibrateZero.servo_id = self.FeetechID

        # Morphing servo setup
        self.MorphingServo = servo_pwm()
        self.MorphingID = 2
        self.MorphingServo.id = self.MorphingID
        self.MorphingPwmMin = 400
        self.MorphingPwmMax = 2400
        self.MorphingPwmShapeX = 1400
        self.MorphingPwmPerchingInBodyX = 400
        self.MorphingPwmPerchingInBodyY = 2400

        # Trigger servo setup
        self.TriggerServo = servo_pwm()
        self.TriggerID = 1
        self.TriggerServo.id = self.TriggerID
        self.TriggerPwmOn = 1250
        self.TriggerPwmOff = 1500

        # self.duration = 1/self.feedback_frequency/len(self.servo_list)
        # rospy.Timer(rospy.Duration(self.duration), self.callback_timer)
        # rospy.Timer(rospy.Duration(duration), PWM_set)


    def callback_gripper_cmd(self, msg):
        # cmdID 1~8 is fixed work way, but 66 is using setpoint position of each servo
        # cmdID = 1: morphing in X shape
        if msg.cmdID == GRIPPER_MORPHING_X:
            self.MorphingServo.pwm = self.MorphingPwmShapeX
            self.pub_servo_pwm.publish(self.MorphingServo)
        # morphing gripper to perching branch in body x
        if msg.cmdID == GRIPPER_MORPHING_BX:
            self.MorphingServo.pwm = self.MorphingPwmPerchingInBodyX
            self.pub_servo_pwm.publish(self.MorphingServo)    
        # morphing gripper to perching branch in body y
        if msg.cmdID == GRIPPER_MORPHING_BY:
            self.MorphingServo.pwm = self.MorphingPwmPerchingInBodyY
            self.pub_servo_pwm.publish(self.MorphingServo)   
        # trigger on 
        if msg.cmdID == GRIPPER_TRIGGER_ON:
            self.TriggerServo.pwm = self.TriggerPwmOn
            self.pub_servo_pwm.publish(self.TriggerServo)
        # trigger off
        if msg.cmdID == GRIPPER_TRIGGER_OFF:
            self.TriggerServo.pwm = self.TriggerPwmOff
            self.pub_servo_pwm.publish(self.TriggerServo)
        # open the gripper
        if msg.cmdID == GRIPPER_OPEN:
            self.FeetechPosition.position = self.FeetechPositionOpen
            self.FeetechSpeed.speed = self.FeetechSpeedFast
            self.pub_feetech_pos.publish(self.FeetechPosition)
            self.pub_feetech_speed.publish(self.FeetechSpeed)
        # slowly close the gripper
        if msg.cmdID == GRIPPER_CLOSE_SLOW:
            self.FeetechPosition.position = self.FeetechPositionClose
            self.FeetechSpeed.speed = self.FeetechSpeedSlow
            self.pub_feetech_pos.publish(self.FeetechPosition)
            self.pub_feetech_speed.publish(self.FeetechSpeed)
        # Fast close the gripper
        if msg.cmdID == GRIPPER_CLOSE_FAST:
            self.FeetechPosition.position = self.FeetechPositionClose
            self.FeetechSpeed.speed = self.FeetechSpeedFast
            self.pub_feetech_pos.publish(self.FeetechPosition)
            self.pub_feetech_speed.publish(self.FeetechSpeed)
        
        # free way 
        if msg.cmdID == GRIPPER_FREE:
            self.FeetechPosition.position = msg.FeetechPosition
            self.pub_feetech_pos.publish(self.FeetechPosition)
            self.FeetechSpeed.speed = msg.FeetechSpeed
            self.pub_feetech_speed.publish(self.FeetechSpeed)
            self.FeetechEnableTorque.enable =  msg.FeetechEnable
            self.pub_feetech_enable.publish(self.FeetechEnableTorque)
            if msg.FeetechZero:
                self.pub_feetech_zero.publish(self.FeetechCalibrateZero)
            self.MorphingServo.pwm = self.angle2pwm(msg.MorphingAngle)
            self.pub_servo_pwm.publish(self.MorphingServo)
            if msg.TriggerOn:
                self.TriggerServo.pwm = msg.TriggerPwmOn
            if not msg.TriggerOn:
                self.TriggerServo.pwm = msg.TriggerPwmOff
            self.pub_servo_pwm.publish(self.TriggerServo)

    
    def angle2pwm(self, angle):
        return (self.MorphingPwmMax-self.MorphingPwmMin)/180 * angle
        
            
    
    def run(self):
        rospy.spin()
        



if __name__ == '__main__':
    try:
        gripper = gripperLM()
        gripper.run()
    except rospy.ROSInterruptException:
        pass