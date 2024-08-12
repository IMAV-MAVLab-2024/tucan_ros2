import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8, Bool
from tucan_msgs.msg import Mode

class MissionDirector(Node):
    """High-level director state machine

    Governs which flight modes are activated and when through a state machine. The state machine publishes 
    integers corresponding to tasks (in the state_dict), and the flight modes activate when their identifier
    is published. When the flight modes are finished, they hand back control to the missiond director through
    publishing a 1 or True on /mode_finished.
    """
    def __init__(self):
        super().__init__('mission_director')
        self.state_publisher = self.create_publisher(Mode, 'mission_state', 10)
        
        self.fm_finish_subscriber = self.create_subscription(Bool,'mode_finished', self.__listener_callback,1)
        
        self.__state = 'start'
        self.__state_dict = {'start': 1,            # Initial state
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

        self.__next_task = 'task_photography' # Assign what the next task to be executed is
        self.__in_control = True # Boolean stating if the mission director has control
        
        self.laps = 0 # Counter for how many laps we have flown
        
        self.frequency = 20 # Node frequency in Hz
        
    def run(self):
        
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            self.__run_state_machine()
            rate.sleep()
        
    def __run_state_machine(self):
        # State machine implementation
        match self.__state:
            case 'start':
                self.__publish_state()           
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition to takeoff
                if self.__in_control:
                    self.__state = 'task_takeoff'
                    self.__in_control = False # Reset control flag

            case 'follow_line':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__state = self.__next_task
                    self.__in_control = False # Reset control flag

            case 'task_photography':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__next_task = 'task_gate'
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag
                
            case 'task_gate':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__next_task = 'task_land'
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag
                
            case 'task_land':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__next_task = 'task_takeoff'
                    self.__state = 'idle'
                    self.__in_control = False # Reset control flag
                
            case 'task_pickup':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__next_task = 'task_place'
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag

            case 'task_place':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__next_task = 'task_window'
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag
                
            case 'task_window':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                if self.__in_control:
                    self.laps += 1 # Increase laps counter when window task is finished
                    self.__next_task = 'task_photography'
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag
                
            case 'task_takeoff':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                # State transition
                if self.__in_control:
                    self.__next_task = 'task_pickup'
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag
                
            case 'find_line':
                self.__publish_state()
                
                self.get_logger().info('State: {self.__state}')
                
                if self.__in_control:
                    self.__state = 'follow_line'
                    self.__in_control = False # Reset control flag
    
    def __publish_state(self):
        """Publish the current state to the mission_state topic
        """
        msg = Mode()
        msg.mode_id = self.__state_dict[int(self.__state)]
        self.state_publisher.publish(msg)
    
    def __listener_callback(self, msg):
        if msg.data == 0:
            self.get_logger().info('Control with flight mode')
            self.__in_control = False
            
        elif msg.data == 1:
            self.get_logger().info('Control with mission director')
            self.__in_control = True
        
        
        
def main(args=None):
    rclpy.init(args=args)
    mission_director = MissionDirector()

    try:
        mission_director.run()
    except KeyboardInterrupt:
        pass
    
    mission_director.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()