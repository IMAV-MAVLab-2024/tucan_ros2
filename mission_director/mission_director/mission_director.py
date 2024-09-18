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
        super().__init__('mission_director_simple')
        # General publishers
        self.mode_publisher = self.create_publisher(Mode, '/active_mode_id', 1)
        self.state_publisher = self.create_publisher(std_msgs.String, '/mission_state', 1)
        self.fm_finish_subscriber = self.create_subscription(ModeStatus,'/mode_status', self.__mode_status_callback, 1)

        # hover publishers
        self.hover_ar_id_pub = self.create_publisher(std_msgs.Int32, 'mode_hover/desired_id', 1)
        self.hover_relative_yaw_pub = self.create_publisher(std_msgs.Float32, 'mode_hover/desired_relative_yaw', 1)
        self.hover_altitude_pub = self.create_publisher(std_msgs.Float32, 'mode_hover/desired_altitude', 1)
        
        # takeoff publishers
        self.takeoff_altitude_pub = self.create_publisher(std_msgs.Float32, '/mode_takeoff/desired_altitude', 1)
        self.takeoff_desired_id_pub = self.create_publisher(std_msgs.Int32, '/mode_takeoff/desired_id', 1)

        self.land_ar_id_pub = self.create_publisher(std_msgs.Int32, 'mode_precision_landing/desired_id', 1)

        # line follower publishers
        self.line_follower_id_pub = self.create_publisher(std_msgs.Int32, 'mode_line_follower/desired_id', 1)
        self.line_follower_altitude_pub = self.create_publisher(std_msgs.Float32, 'mode_line_follower/desired_altitude', 1)

        # Task publishers
        self.task_photography_id_pub = self.create_publisher(std_msgs.Int32, 'mode_photographer/desired_id', 1)
        self.task_photography_altitude_pub = self.create_publisher(std_msgs.Float32, 'mode_photographer/desired_altitude', 1)
        self.task_photography_yaw_pub = self.create_publisher(std_msgs.Float32, 'mode_photographer/desired_yaw', 1)

        # Gate publishers
        self.task_gate_id_pub = self.create_publisher(std_msgs.Int32, 'mode_gate/desired_id', 1)

        self.start_start_time = None

        self.currently_active_mode_id = Mode.NO_MODE

        self.frequency = 4             # state machine frequency, also gets run whenever mode feedback is received

        self.mode_feedback_ = ModeStatus()

        self.__state = "start"

        self.timer = self.create_timer(1/self.frequency, self.__run_state_machine)

        self.marker_ids = {
            'start': 100, # Start
            'photography': 105, # Photography
            'gate_start': 200, # Before gate
            'gate_end': 205, # After gate
            'platform_50': 300, # 50 cm platform
            'platform_35': 301, # 35 cm platform
            'platform_20': 302, # 20 cm platform
            'sample': 400, # Sample
            'placement': 405 # Placement target
        }
        self.desired_relative_yaw_dict = {
            'start': math.pi, # Start
            'before_photography': math.pi/2., # Photography
            'after_photography': -math.pi/2., # Photography
            'before_gate': -math.pi/2., # Before gate
            'after_gate': -math.pi/2., # After gate
            'platform_50': math.pi, # 50 cm platform
            'platform_35': -math.pi/2., # 35 cm platform
            'platform_20': -math.pi/2, # 20 cm platform, to face the sample
            'sample': 0., # Sample
            'placement': -math.pi/2. # Placement target
        }

        self.hover_time = 5
        self.hover_altitude = 1.5
        self.wait_time = 10


    def __run_state_machine(self):
        # State machine implementation
        match self.__state:
            case 'start':
                self.__state = 'takeoff_start'
                self.get_logger().info(f'Switching to {self.__state} mode')
                self.start_start_time = time.time()
                
            case 'takeoff_start':
                self.currently_active_mode_id = Mode.TAKEOFF
                self.takeoff_desired_id_pub.publish(std_msgs.Int32(data=self.marker_ids['start']))
                self.takeoff_altitude_pub.publish(std_msgs.Float32(data=float(1.5)))

                if self.mode_feedback_.mode.mode_id == Mode.TAKEOFF and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_start'
                    self.get_logger().info(f'Takeoff finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_start':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['start'])) # Hover over the start marker
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=float(self.desired_relative_yaw_dict['start'])))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=float(self.hover_altitude)))
                self.currently_active_mode_id = Mode.HOVER

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'line_follower_to_photography'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_photography':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography'])) # Follow the line to the photography marker
                self.line_follower_altitude_pub(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER

                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_before_photography'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_before_photography':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography'])) # Stay above photography marker
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['before_photography']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=float(self.hover_altitude)))
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'photography'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'photography':
                self.task_photography_yaw_pub.publish(std_msgs.Float32(data=float(self.desired_relative_yaw_dict['before_photography'])))
                self.task_photography_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography']))
                self.task_photography_altitude_pub.publish(std_msgs.Float32(data=float(self.hover_altitude)))
                self.currently_active_mode_id = Mode.WILDLIFE_PHOTOGRAPHER

                if self.mode_feedback_.mode.mode_id == Mode.WILDLIFE_PHOTOGRAPHER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_after_photography'
                    self.get_logger().info(f'Photography finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_after_photography':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography'])) # Stay above photography marker
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['after_photography']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=float(self.hover_altitude)))
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'line_follower_to_gate_start'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_gate_start':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_start'])) # Follow the line to the photography marker
                self.line_follower_altitude_pub(std_msgs.Float32(data=float(1.75)))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  

                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_gate_start'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()
                    self.start_start_time = time.time()

            case 'hover_gate_start':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_start'])) # Stay above photography marker
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['before_gate']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=1.75)) # middle gate
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'gate'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'gate':
                self.task_gate_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_end'])) # publish the marker id after the gate
                self.currently_active_mode_id = Mode.VERTICAL_GATE
                if self.mode_feedback_.mode.mode_id == Mode.VERTICAL_GATE and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_gate_end'
                    self.get_logger().info(f'Gate finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_gate_end':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_end'])) # Hover over after gate marker
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['after_gate']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=1.75)) # middle gate
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'line_follower_to_large_platform'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_large_platform':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_50'])) # Follow the line to the specified land marker
                self.line_follower_altitude_pub(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_large_platform'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_large_platform':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_50']))
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['platform_50'])) 
                self.hover_altitude_pub.publish(std_msgs.Float32(data=self.hover_altitude)) # Go to 2.5 meter so we can see the other platforms
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'hover_middle_platform'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_middle_platform':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35'])) # Follow the line to the specified land marker
                self.line_follower_altitude_pub(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_middle_platform'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_middle_platform':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35']))
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['platform_35']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=1.5))             
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'land'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'land':
                self.land_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35']))
                self.currently_active_mode_id = Mode.PRECISION_LANDING  

                if self.mode_feedback_.mode.mode_id == Mode.PRECISION_LANDING and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'wait'
                    self.get_logger().info(f'Landing finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'wait':
                if time.time() - self.start_start_time > self.wait_time:
                    self.__state = 'arm'
                    self.get_logger().info(f'Wait finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'arm':
                self.currently_active_mode_id = Mode.ARM

                if self.mode_feedback_.mode.mode_id == Mode.ARM and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'takeoff_after_landing'
                    self.get_logger().info(f'Arm finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'takeoff_after_landing':
                self.currently_active_mode_id = Mode.TAKEOFF

                if self.mode_feedback_.mode.mode_id == Mode.TAKEOFF and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_after_landing'
                    self.get_logger().info(f'Takeoff finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_after_landing':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35']))
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['platform_35']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=2.5))
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'line_follower_to_small_platform'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_small_platform':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35'])) # Follow the line to the specified land marker
                self.line_follower_altitude_pub.publish(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  

                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_small_platform'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_small_platform':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_20']))
                self.hover_relative_yaw_pub.publish(std_msgs.Float32(data=self.desired_relative_yaw_dict['platform_20']))
                self.hover_altitude_pub.publish(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.HOVER  

                if time.time() - self.start_start_time > self.hover_time:
                    self.__state = 'line_follower_to_sample'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            # ----- CHANGE HERE TO SKIP SAMPLE -----
            case 'line_follower_to_sample':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['sample'])) # Follow the line to the sample
                self.line_follower_altitude_pub.publish(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  

                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:

                    self.__state = 'line_follower_to_placement'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_placement':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['placement']))
                self.line_follower_altitude_pub.publish(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  

                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    #self.__state = 'place_sample'
                    self.__state = 'line_follower_to_start'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_start':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['start']))
                self.line_follower_altitude_pub.publish(std_msgs.Float32(data=self.hover_altitude))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  

                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'land_on_start'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'land_on_start':
                self.land_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['start']))
                self.currently_active_mode_id = Mode.PRECISION_LANDING  

                if self.mode_feedback_.mode.mode_id == Mode.PRECISION_LANDING and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'mission_finished'
                    self.get_logger().info(f'Landing finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'mission_finished':
                self.get_logger().info(f'MISSION ENDED')
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
    mission_director = MissionDirector()

    rclpy.spin(mission_director)
    
    mission_director.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()