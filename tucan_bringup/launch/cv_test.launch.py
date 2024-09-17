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
    
    # Drivers - Front camera
    # Drivers - Downward camera
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

    # Vision nodes
    ar_detection_node = Node(
        package='cv_aruco_detector',
        executable='cv_aruco_detector',
    )
    line_detection_node = Node(
        package='cv_line_detector',
        executable='cv_line_detector',
        parameters=[
            {"debug": True}
        ],
    )
    
    # Offboard handler
    offboard_handler = Node(
        package='offboard_handler',
        executable='offboard_handler',
    )

    line_mode = Node(
        package='mode_line_follower',
        executable='mode_line_follower',
    )

    # Mission director - simple experiment edition
    mission_director = Node(
        package='mission_director',
        executable='mission_director_simple',
    )

    # Add all the actions
    ld.add_action(line_mode)
    ld.add_action(front_camera_driver_node)
    ld.add_action(down_camera_driver_node)
    
    ld.add_action(ar_detection_node)
    ld.add_action(line_detection_node)

    ld.add_action(offboard_handler)

    return ld