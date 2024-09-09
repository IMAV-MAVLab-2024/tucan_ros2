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
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[
            {"video_device": "/dev/video22"},
            {"time_per_frame": [1, 30]},
            {"pixel_format": "UYVY"}
        ],
        remappings=[
            ('/image_raw', '/down_camera_image'),
            ('/image_raw/compressed', '/down_camera_image/compressed')
        ],
        name='down_cam_driver'
    )
    
    # Offboard handler
    offboard_handler = Node(
        package='offboard_handler',
        executable='offboard_handler',
    )

    # Add all the actions
    ld.add_action(down_camera_driver_node)

    ld.add_action(offboard_handler)

    return ld