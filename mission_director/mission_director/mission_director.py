import rclpy
from rclpy.node import Node

import time

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
        self.mode_publisher = self.create_publisher(Mode, '/active_mode_id', 1)
        self.state_publisher = self.create_publisher(std_msgs.String, '/mission_state', 1)
        self.fm_finish_subscriber = self.create_subscription(ModeStatus,'/mode_status', self.__mode_status_callback, 1)

        self.hover_ar_id_pub = self.create_publisher(std_msgs.Int32, 'mode_hover/desired_id', 1)
        self.land_ar_id_pub = self.create_publisher(std_msgs.Int32, 'mode_precision_landing/desired_id', 1)
        self.line_follower_id_pub = self.create_publisher(std_msgs.Int32, 'mode_line_follower/desired_id', 1)
        self.task_photography_id_pub = self.create_publisher(std_msgs.Int32, 'mode_photography/desired_id', 1)
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


    def __run_state_machine(self):
        # State machine implementation
        match self.__state:
            case 'start':
                self.__state = 'takeoff_start'
                self.get_logger().info(f'Switching to {self.__state} mode')
                
            case 'takeoff_start':
                self.currently_active_mode_id = Mode.TAKEOFF

                if self.mode_feedback_.mode.mode_id == Mode.TAKEOFF and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'line_follower'
                    self.get_logger().info(f'Takeoff finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_start':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['start'])) # Hover over the start marker
                self.currently_active_mode_id = Mode.HOVER  
                #run for 20 seconds
                if time.time() - self.start_start_time > 5:
                    self.__state = 'line_follower_to_photography'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')

            case 'line_follower_to_photography':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography'])) # Follow the line to the photography marker
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_photography'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_photography':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography'])) # Stay above photography marker
                self.currently_active_mode_id = Mode.HOVER  
                #run for 20 seconds
                if time.time() - self.start_start_time > 10:
                    self.__state = 'photography'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')

            case 'photography':
                self.task_photography_id_pub.publish(std_msgs.Int32(data=self.marker_ids['photography']))
                self.currently_active_mode_id = Mode.PHOTOGRAPHY
                if self.mode_feedback_.mode_id == Mode.PHOTOGRAPHY and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'line_follower_to_gate_start'
                    self.get_logger().info(f'Photography finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_gate_start':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_start'])) # Follow the line to the photography marker
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_gate_start'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_gate_start':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_start'])) # Stay above photography marker
                self.currently_active_mode_id = Mode.HOVER  
                #run for 10 seconds
                if time.time() - self.start_start_time > 10:
                    self.__state = 'gate'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')

            case 'gate':
                self.task_gate_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_end'])) # publish the marker id after the gate
                self.currently_active_mode_id = Mode.GATE
                if self.mode_feedback_.mode_id == Mode.GATE and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_gate_end'
                    self.get_logger().info(f'Gate finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_gate_end':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['gate_end'])) # Hover over after gate marker
                self.currently_active_mode_id = Mode.HOVER  
                #run for 10 seconds
                if time.time() - self.start_start_time > 10:
                    self.__state = 'line_follower_to_platform_landing'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')

            case 'line_follower_to_platform_landing':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35'])) # Follow the line to the specified land marker
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_before_precision_landing'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'hover_before_precision_landing':
                self.hover_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35']))                           
                self.currently_active_mode_id = Mode.HOVER  
                #run for 10 seconds
                if time.time() - self.start_start_time > 10:
                    self.__state = 'land'
                    self.get_logger().info(f'Hover finished, switching to: {self.__state}')

            case 'land':      
                self.land_ar_id_pub.publish(std_msgs.Int32(data=self.marker_ids['platform_35']))
                self.currently_active_mode_id = Mode.PRECISION_LANDING  

                if self.mode_feedback_.mode.mode_id == Mode.PRECISION_LANDING and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'wait'
                    self.get_logger().info(f'Landing finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'wait':      
                if time.time() - self.start_start_time > 5:
                    self.__state = 'arm'
                    self.get_logger().info(f'Wait finished, switching to: {self.__state}')

            case 'arm':      
                self.currently_active_mode_id = Mode.ARM  

                if self.mode_feedback_.mode.mode_id == Mode.ARM and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'takeoff_after_landing'
                    self.get_logger().info(f'Arm finished, switching to: {self.__state}')
                
            case 'takeoff_after_landing':
                self.currently_active_mode_id = Mode.TAKEOFF

                if self.mode_feedback_.mode.mode_id == Mode.TAKEOFF and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'line_follower_to_sample'
                    self.get_logger().info(f'Takeoff finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'line_follower_to_sample':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['sample'])) # Follow the line to the sample
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'pickup_sample'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()
            
            case 'pickup_sample':
                self.currently_active_mode_id = Mode.PICKUP_SAMPLE

                if self.mode_feedback_.mode.mode_id == Mode.PICKUP_SAMPLE and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'line_follower_to_placement'
                    self.get_logger().info(f'Pickup finished, switching to: {self.__state}')
                    self.start_start_time = time.time()
            
            case 'line_follower_to_placement':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['placement']))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'place_sample'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()

            case 'place_sample':
                self.currently_active_mode_id = Mode.PLACE_SAMPLE

                if self.mode_feedback_.mode.mode_id == Mode.PLACE_SAMPLE and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'line_follower_to_start'
                    self.get_logger().info(f'Place finished, switching to: {self.__state}')
                    self.start_start_time = time.time()
            
            case 'line_follower_to_start':
                self.line_follower_id_pub.publish(std_msgs.Int32(data=self.marker_ids['start']))
                self.currently_active_mode_id = Mode.LINE_FOLLOWER  
                if self.mode_feedback_.mode.mode_id == Mode.LINE_FOLLOWER and self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover_start'
                    self.get_logger().info(f'Line_follower finished, switching to: {self.__state}')
                    self.start_start_time = time.time()                
            
            case 'mission_finished':
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