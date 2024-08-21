import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
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
        self.state_publisher = self.create_publisher(Mode, '/mission_state', 10)

        self.fm_finish_subscriber = self.create_subscription(ModeStatus,'/mode_status', self.__listener_callback,1)

        self.__state = 'idle'
        self.__state_dict = {'hover': 1,            # Hover over an ArUco marker
                             'follow_line': 2,      # Follow line until next ArUco marker
                             'task_photography': 3, # Execute wildlife photography task
                             'task_gate': 4,        # Execute gate passing task
                             'task_land': 5,        # Execute landing task
                             'task_pickup': 6,      # Execute pickup task
                             'task_place': 7,       # Execute place task
                             'task_window': 8,      # Execute window task (implementation determines do or avoid)
                             'task_takeoff': 9,     # Do a takeoff
                             'find_line': 10,        # Default state if no line is detected or task is active
                             'idle': 0}            # Do-nothing state

        self.__next_task = 'task_land' # Assign what the next task to be executed is
        self.__transition = False # Boolean stating if state transitioning is enabled
        
        self.__from_follow_line = False # Boolean stating if the mission director is coming from follow line

        self.laps = 0 # Counter for how many laps we have flown

        self.__frequency = 1. # Node frequency in Hz
        self.timer = self.create_timer(1./self.__frequency, self.timer_callback)

    def timer_callback(self):
        self.__publish_state()

    def __run_state_machine(self):
        # State machine implementation
        match self.__state:
            case 'idle':
                self.__publish_state()
                self.get_logger().info(f'State: {self.__state}')

                if self.__transition:
                    self.__state = 'task_takeoff'
                    self.__transition = False

            case 'hover':
                self.__publish_state()           
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    if self.__from_follow_line:
                        self.__state = self.__next_task
                        self.__from_follow_line = False
                    else:
                        self.__state = 'follow_line'
                    self.__transition = False # Reset control flag

            case 'follow_line':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__state = 'hover'
                    self.__from_follow_line = True
                    self.__transition = False # Reset control flag

            case 'task_photography':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__next_task = 'task_gate'
                    self.__state = 'hover'
                    self.__transition = False # Reset control flag
                
            case 'task_gate':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__next_task = 'task_land'
                    self.__state = 'follow_line'
                    self.__transition = False # Reset control flag
                
            case 'task_land':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__next_task = 'task_takeoff'
                    self.__state = 'idle'
                    self.__transition = False # Reset control flag
                
            case 'task_pickup':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__next_task = 'task_place'
                    self.__state = 'follow_line'
                    self.__transition = False # Reset control flag

            case 'task_place':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__next_task = 'task_window'
                    self.__state = 'follow_line'
                    self.__transition = False # Reset control flag
                
            case 'task_window':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                if self.__transition:
                    self.laps += 1 # Increase laps counter when window task is finished
                    self.__next_task = 'task_photography'
                    self.__state = 'follow_line'
                    self.__transition = False # Reset control flag
                
            case 'task_takeoff':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                # State transition
                if self.__transition:
                    self.__state = 'hover'
                    self.__transition = False # Reset control flag
                
            case 'find_line':
                self.__publish_state()
                
                self.get_logger().info(f'State: {self.__state}')
                
                if self.__transition:
                    self.__state = 'follow_line'
                    self.__transition = False # Reset control flag
    
    def __publish_state(self):
        """Publish the current state to the mission_state topic
        """
        msg = Mode()
        msg.mode_id = int(self.__state_dict[self.__state])
        self.state_publisher.publish(msg)
    
    def __listener_callback(self, msg):
        # MODE_ERROR = 0         # Something went wrong
        # MODE_INACTIVE = 1      # Mode is inactive
        # MODE_ACTIVE = 2        # Mode is active
        # MODE_FINISHED = 3      # Mode has finished
        status = msg.mode_status
        match status:
            case 0: # Land in case of error
                self.__transition = False
                self.__state = 'land'
                # Run state machine to go to land state
                self.__run_state_machine()
            
            case 1:
                self.__transition = False
                self.__state = 'land'
                self.__run_state_machine()
            
            case 2:
                self.__transition = False
            
            case 3:
                self.get_logger().info('Mode finished - control to MD')
                # Run state machine while allowing to transition to the next state
                self.__transition = True
                self.__run_state_machine()
        
def main(args=None):
    rclpy.init(args=args)
    mission_director = MissionDirector()

    rclpy.spin(mission_director)
    
    mission_director.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()