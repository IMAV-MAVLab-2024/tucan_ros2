from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for testing the CV pipeline. This launch file launches the camera drivers and cv nodes, along with a flight mode to be tested
using the CV pipeline.
To add nodes to be launched, copy the code here and add the package containing the node as a dependency in the package.xml file.

The package can be launched with 'ros2 launch tucan_bringup test_cv.launch.py'
"""

def generate_launch_description():
    ld = LaunchDescription()
    
    # === DRIVERS ===
    # Drivers - Front camera
    down_camera_driver_node = Node(
        package='driver_camera',
        executable='camera_driver_node',
        parameters=[
            {"camera_id": 22},
            {"compress": False},
            {"FPS": 30},
            {"frame_width": 720},
            {"frame_height": 576},
            {"topic_name": "/down_camera_image"}
        ],
        name='down_cam_driver'
    )

    # Drivers - Forward camera
    front_camera_driver_node = Node(
        package='driver_camera',
        executable='camera_driver_node',
        parameters=[
            {"camera_id": 31},
            {"compress": False},
            {"FPS": 2},
            {"frame_width": 720},
            {"frame_height": 576},
            {"topic_name": "/front_camera_image"}
        ],
        name='front_cam_driver'
    )

    # === VISION ===
    ar_detection_node = Node(
        package='cv_aruco_detector',
        executable='cv_aruco_detector',
    )
    line_detection_node = Node(
        package='cv_line_detector',
        executable='cv_line_detector',
    )
    
    # === FLIGHT MODES ===
    arm_node = Node(
        package='mode_arm',
        executable='mode_arm',
    )
    hover_node = Node(
        package='mode_hover',
        executable='mode_hover',
    )    
    line_follow_node = Node(
        package='mode_line_follower',
        executable='mode_line_follower',
    )
    photography_node = Node(
        package='mode_wildlife_photographer',
        executable='mode_wildlife_photographer',
    )    
    gate_node = Node(
        package='mode_vertical_gate',
        executable='mode_vertical_gate',
    )    
    land_node = Node(
        package='mode_precision_landing',
        executable='mode_precision_landing',
    )
    takeoff_node = Node(
        package='mode_takeoff',
        executable='mode_takeoff',
        parameters=[
            {"which_gate": 'middle'}
        ],
    )        
        
    # === MISSION DIRECTOR ===
    mission_director = Node(
        package='mission_director',
        executable='mission_director',
    )
    # Offboard handler
    offboard_handler = Node(
        package='offboard_handler',
        executable='offboard_handler',
    )

    # Add all the actions
    ld.add_action(front_camera_driver_node)
    ld.add_action(down_camera_driver_node)
    
    ld.add_action(ar_detection_node)
    ld.add_action(line_detection_node)
    
    ld.add_action(arm_node)
    ld.add_action(hover_node)
    ld.add_action(line_follow_node)
    ld.add_action(photography_node)
    ld.add_action(gate_node)
    ld.add_action(land_node)
    ld.add_action(takeoff_node)

    ld.add_action(mission_director)
    ld.add_action(offboard_handler)

    return ld