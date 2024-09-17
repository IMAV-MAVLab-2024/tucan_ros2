import rclpy
from rclpy.node import Node

import time

import math

import std_msgs.msg as std_msgs
from tucan_msgs.msg import Mode, ModeStatus

class MissionDirector(Node):
    """High-level director state machine for the simple line follow experiment

    Governs which flight modes are activated and when through a state machine. The state machine publishes 
    integers corresponding to tasks (in the state_dict), and the flight modes activate when their identifier
    is published. When the flight modes are finished, they hand back control to the missiond director through
    publishing a 1 or True on /mode_finished.
    """
    def __init__(self):
        super().__init__('mission_director_AHT')
        self.mode_publisher = self.create_publisher(Mode, '/active_mode_id', 1)
        self.state_publisher = self.create_publisher(std_msgs.String, '/mission_state', 1)
        self.fm_finish_subscriber = self.create_subscription(ModeStatus,'/mode_status', self.__mode_status_callback, 1)

        self.hover_desired_yaw_pub = self.create_publisher(std_msgs.Float32, '/mode_hover/desired_relative_yaw', 1)
        self.hover_altitude_pub = self.create_publisher(std_msgs.Float32, 'mode_hover/desired_altitude', 1)

        self.hover_ar_id_pub = self.create_publisher(std_msgs.Int32, '/mode_hover/desired_id', 1)
        self.land_ar_id_pub = self.create_publisher(std_msgs.Int32, '/mode_precision_landing/desired_id', 1)
        self.line_follower_id_pub = self.create_publisher(std_msgs.Int32, '/mode_line_follower/desired_id', 1)

        self.takeoff_altitude_pub = self.create_publisher(std_msgs.Float32, '/mode_takeoff/desired_altitude', 1)

        self.land_desired_pub = self.create_publisher(std_msgs.Float32, '/mode_precision_landing/desired_relative_yaw', 1)

        self.photographer_ar_id_pub = self.create_publisher(std_msgs.Int32, '/mode_photographer/desired_id',1)

        self.start_time = None

        self.currently_active_mode_id = Mode.NO_MODE

        self.frequency = 4             # state machine frequency, also gets run whenever mode feedback is received

        self.mode_feedback_ = ModeStatus()

        self.__state = "start"

        self.timer = self.create_timer(1/self.frequency, self.__run_state_machine)

        self.start_marker_id = 100
        self.end_marker_id = 105


    def __run_state_machine(self):
        # State machine implementation
        match self.__state:
            case 'start':
                self.__state = 'takeoff'
                self.get_logger().info(f'Switching to {self.__state} mode')
                
            case 'takeoff':
                self.currently_active_mode_id = Mode.TAKEOFF
                self.takeoff_altitude_pub.publish(std_msgs.Float32(data=float(1.0)))

                if self.mode_feedback_.mode.mode_id == Mode.TAKEOFF and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_stay'
                    self.get_logger().info(f'Takeoff finished, switching to: {self.__state}')
                    self.start_time = time.time()

            case 'hover_stay':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.start_marker_id))
                self.hover_desired_yaw_pub.publish(std_msgs.Float32(data=float(math.pi)))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=float(1.0)))
                self.currently_active_mode_id = Mode.HOVER

                #run for 10 seconds
                if time.time() - self.start_time > 10:
                    self.__state = 'line_follower'
                    self.get_logger().info(f'hover_stay finished, switching to: {self.__state}')
                    self.start_time = time.time()

            case 'line_follower':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.end_marker_id))
                self.line_follower_altitude_pub.publish(std_msgs.Float32(data=1.0))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_end'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_time = time.time()
            
            case 'hover_end':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.end_marker_id))
                self.hover_desired_yaw_pub.publish(std_msgs.Float32(data=float(math.pi)))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=float(0.5)))
                self.currently_active_mode_id = Mode.HOVER 

                #run for 10 seconds
                if time.time() - self.start_time > 10:
                    self.__state = 'land'
                    self.get_logger().info(f'hover_end finished, switching to: {self.__state}')
                    self.start_time = time.time()                

            case 'land':      
                self.currently_active_mode_id = Mode.PRECISION_LANDING  

                if self.mode_feedback_.mode.mode_id == Mode.PRECISION_LANDING and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'mission_finished'
                    self.get_logger().info(f'Landing finished, switching to: {self.__state}')
                    self.start_time = time.time()        

            case 'mission_finished':
                self.get_logger().info('Congrats, mission completed!')
                pass

        
        if self.currently_active_mode_id is not None:
            #self.get_logger().debug(f'Currently active mode: {self.currently_active_mode_id}')
            msg = Mode()
            msg.mode_id = self.currently_active_mode_id
            self.mode_publisher.publish(msg)

        self.__publish_state()
    
    def __publish_state(self):
        """Publish the current state to the mission_state topic
        """
        msg = std_msgs.String()
        msg.data = self.__state
        self.state_publisher.publish(msg)
    
    def __mode_status_callback(self, msg):
        # MODE_ERROR = 0         # Something went wrong
        # MODE_INACTIVE = 1      # Mode is inactive
        # MODE_ACTIVE = 2        # Mode is active
        # MODE_FINISHED = 3      # Mode has finished

        if msg.mode.mode_id == self.currently_active_mode_id:
            self.mode_feedback_ = msg
            self.__run_state_machine()
        
def main(args=None):
    rclpy.init(args=args)
    mission_director_AHT = MissionDirector()

    rclpy.spin(mission_director_AHT)
    
    mission_director_AHT.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
