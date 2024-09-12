import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import Bool, String
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
        self.state_publisher = self.create_publisher(String, '/mission_state', 1)
        self.fm_finish_subscriber = self.create_subscription(ModeStatus,'/mode_status', self.__mode_status_callback, 1)

        self.currently_active_mode_id = Mode.IDLE

        self.frequency = 2             # state machine frequency, also gets run whenever mode feedback is received

        self.mode_feedback_ = ModeStatus()

        self.__state = "start"

        self.timer = self.create_timer(1/self.frequency, self.__run_state_machine)


    def __run_state_machine(self):
        # State machine implementation
        match self.__state:
            case 'start':
                self.get_logger().info(f'Switching to hover mode')
                self.__state = 'hover'
                
            case 'takeoff':
                self.currently_active_mode_id = Mode.TAKEOFF

                if self.mode_feedback_.mode_status == ModeStatus.MODE_FINISHED:
                    self.__state = 'hover'
                    self.get_logger().info(f'Takeoff finished, switching to: {self.__state}')

            case 'hover':      
                self.currently_active_mode_id = Mode.HOVER  
        
        if self.currently_active_mode_id is not None:
            #self.get_logger().debug(f'Currently active mode: {self.currently_active_mode_id}')
            msg = Mode()
            msg.mode_id = self.currently_active_mode_id
            self.mode_publisher.publish(msg)

        self.__publish_state()
    
    def __publish_state(self):
        """Publish the current state to the mission_state topic
        """
        msg = String()
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