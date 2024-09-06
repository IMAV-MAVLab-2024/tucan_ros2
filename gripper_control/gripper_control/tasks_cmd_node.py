import rospy
from tucan_msgs.msg import servo_feedback
from tucan_msgs.msg import servo_speed
from tucan_msgs.msg import servo_position
from tucan_msgs.msg import servo_pwm
from tucan_msgs.msg import servo_enable_torque
from tucan_msgs.msg import servo_calibrate_zero
from tucan_msgs.msg import gripper_cmd

import scripts.teensy_python_interface as teensy
import time

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

def tester():
    rospy.init_node('tasks_cmd_node', anonymous=True)
    pub_speed = rospy.Publisher('/gripper/command', gripper_cmd, queue_size=10)

    time.sleep(.5)

    #msg = servo_speed()
    #msg.servo_id = 9
    #msg.speed = 2000

    #pub_speed.publish(msg)


    #rospy.loginfo(("Set speed command in tester:" +  str(msg.speed) + "for servo:" + str(msg.servo_id)))

    # for i in range(1, 9):
    #     msg = servo_calibrate_zero()
    #     msg.servo_id = i

    #     pub_zero.publish(msg)

    #     rospy.loginfo(("Set zero command in tester for servo:" + str(msg.servo_id)))

    # return
    
    rate = rospy.Rate(20) # 10hz

    servo_step = 100
    motor_step = 10
    current_pos_servo = 100
    current_pwm = 1000
    current_pos_motor1 = 1000
    current_pos_motor2 = 1000
    id = 3
    

    gripper = gripper_cmd()
    while not rospy.is_shutdown():
        # current_pos_servo += servo_step
        gripper.cmdID = GRIPPER_MORPHING_BY
        pub_speed.publish(gripper)
       

        rate.sleep()




if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass