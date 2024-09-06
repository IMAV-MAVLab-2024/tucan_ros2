import rospy
from gripper_control.msg import servo_feedback
from gripper_control.msg import servo_speed
from gripper_control.msg import servo_position
from gripper_control.msg import servo_pwm
from gripper_control.msg import servo_enable_torque
from gripper_control.msg import servo_calibrate_zero
import scripts.teensy_python_interface as teensy
import time

def tester():
    rospy.init_node('gripper_test_node', anonymous=True)
    pub_speed = rospy.Publisher('servo_set_speed', servo_speed, queue_size=10)
    pub_pos = rospy.Publisher('servo_set_position', servo_position, queue_size=10)
    pub_servo_pwm = rospy.Publisher('servo_pwm', servo_pwm, queue_size=10)
    pub_enable = rospy.Publisher('servo_enable_torque', servo_enable_torque, queue_size=10)
    pub_zero = rospy.Publisher('servo_set_zero_position', servo_calibrate_zero, queue_size=10)

    time.sleep(.5)

    #msg = servo_speed()
    #msg.servo_id = 9
    #msg.speed = 2000

    #pub_speed.publish(msg)


    #rospy.loginfo(("Set speed command in tester:" +  str(msg.speed) + "for servo:" + str(msg.servo_id)))

    for i in range(1, 9):
        msg = servo_calibrate_zero()
        msg.servo_id = i

        pub_zero.publish(msg)

        rospy.loginfo(("Set zero command in tester for servo:" + str(msg.servo_id)))

    # return
    
    rate = rospy.Rate(20) # 10hz

    servo_step = 100
    motor_step = 10
    current_pos_servo = 100
    current_pwm = 1000
    current_pos_motor1 = 1000
    current_pos_motor2 = 1000
    id = 3
    


    while not rospy.is_shutdown():
        current_pos_servo += servo_step
        if current_pos_servo >= 4000:
            current_pos_servo = 100
        current_pwm += motor_step
        if current_pwm > 2400:
            current_pwm = 400

        # morphing [400,2400]
        current_pwm_morphing = current_pwm
        # trigger [1250, 1500]
        current_pwm_trigger = 1300

        msg = servo_position()
        msg.servo_id = 3
        msg.position = current_pos_servo
        pub_pos.publish(msg)

        servo_trigger_msg = servo_pwm()
        servo_trigger_msg.motor_id = 1
        servo_trigger_msg.pwm = current_pwm_trigger
        pub_servo_pwm.publish(servo_trigger_msg)

        servo_morphing_msg = servo_pwm()
        servo_morphing_msg.motor_id = 2
        servo_morphing_msg.pwm = current_pwm_morphing
        pub_servo_pwm.publish(servo_morphing_msg)

        


        rate.sleep()

 


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass