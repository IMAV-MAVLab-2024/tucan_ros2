from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'servo_list',
            default_value='[3]',
            description='List of servo motors'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the gripper'
        ),
        DeclareLaunchArgument(
            'feedback_frequency',
            default_value='3.0',
            description='Feedback frequency for the gripper'
        ),

        # Define the node with its parameters
        Node(
            package='gripper_control',
            executable='gripper_active_node.py',
            name='gripper_active_node',
            output='screen',
            parameters=[{
                'servo_list': LaunchConfiguration('servo_list'),
                'serial_port': LaunchConfiguration('serial_port'),
                'feedback_frequency': LaunchConfiguration('feedback_frequency'),
            }]
        ),
    ])
