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
    

    # Drivers - Downward camera
    down_camera_driver_node = Node(
        package='driver_camera',
        executable='camera_driver_node',
        parameters=[
            {"camera_id": 22},
            {"compress": False},
            {"FPS": 100},
            {"frame_width": 600},
            {"frame_height": 400},
            {"topic_name": "/down_camera_image"}
        ],
        name='down_cam_driver'
    )

    # Drivers - Forward camera
    # front_camera_driver_node = Node(
    #     package='driver_camera',
    #     executable='camera_driver_node',
    #     parameters=[
    #         {"camera_id": 31},
    #         {"compress": False},
    #         {"FPS": 2},
    #         {"frame_width": 800},
    #         {"frame_height": 600},
    #         {"topic_name": "/front_camera_image"}
    #     ],
    #     name='front_cam_driver'
    # )
    
    # Offboard handler
    offboard_handler = Node(
        package='offboard_handler',
        executable='offboard_handler',
    )

    # Add all the actions
    ld.add_action(down_camera_driver_node)
    #ld.add_action(front_camera_driver_node)
    ld.add_action(offboard_handler)

    return ld