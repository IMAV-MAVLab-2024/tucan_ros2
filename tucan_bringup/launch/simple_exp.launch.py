from launch import LaunchDescription
from launch_ros.actions import Node

"""
Simple experiment launch file. This launch file launches the necessary node to do the initial experiment,
defines as takeoff -> line follow -> hover -> land.
To add nodes to be launched, copy the code here and add the package containing the node as a dependency in the package.xml file.

The package can be launched with 'ros2 launch tucan_bringup simple_exp.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()
    
    # Drivers
    camera_driver_node = Node(
        package='driver_camera',
        executable='camera_driver_node',
    )
    
    # Vision nodes
    ar_detection_node = Node(
        package='cv_aruco_detector',
        executable='cv_aruco_detector',
    )
    line_detection_node = Node(
        package='cv_line_detector',
        executable='cv_line_detector',
    )
    
    # Flight mode nodes
    idle_node = Node(
        package='mode_idle',
        executable='mode_idle',
    )
    takeoff_node = Node(
        package='mode_takeoff',
        executable='mode_takeoff',
    )
    line_follow_node = Node(
        package='mode_line_follower',
        executable='mode_line_follower',
    )
    hover_node = Node(
        package='mode_hover',
        executable='mode_hover',
    )
    land_node = Node(
        package='mode_precision_landing',
        executable='mode_precision_landing',
    )
    
    # Mission director - simple experiment edition
    mission_director = Node(
        package='mission_director',
        executable='mission_director_simple',
    )
    
    # Offboard handler
    offboard_handler = Node(
        package='offboard_handler',
        executable='offboard_handler',
    )
    
    # Add all the actions
    ld.add_action(camera_driver_node)
    
    ld.add_action(ar_detection_node)
    ld.add_action(line_detection_node)
    
    ld.add_action(idle_node)
    ld.add_action(takeoff_node)
    ld.add_action(line_follow_node)
    ld.add_action(hover_node)
    ld.add_action(land_node)
    
    ld.add_action(mission_director)
    ld.add_action(offboard_handler)
    
    return ld