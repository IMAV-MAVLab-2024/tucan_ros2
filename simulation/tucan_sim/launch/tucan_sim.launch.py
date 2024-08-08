# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
import signal
import subprocess

# Signal handler for SIGINT
def sigint_handler(signum, frame):
    subprocess.run(['pgrep -f QGroundControl | xargs kill -9'], shell=True)
    subprocess.run(['pgrep -f px4 | xargs kill -9'], shell=True)
    #sys.exit(0)

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    use_groundcontrol = DeclareLaunchArgument('groundcontrol', default_value='true',
                                              choices=['true', 'false'],
                                              description='Start ground control station.')

    world_pkgs = get_package_share_directory('tucan_sim')
    gateway_models_dir = get_package_share_directory('tucan_sim')

    os.environ['PX4_GZ_MODEL_POSE'] =  '0, 0, .2, 0, 0, 0'
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(world_pkgs, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gateway_models_dir, 'models')

    wait_spawn = ExecuteProcess(cmd=["sleep", "5"])
    
    px4_script_path = os.path.join(os.path.dirname(__file__), 'px4.launch.sh')
    microxrce_script_path = os.path.join(os.path.dirname(__file__), 'microxrce.launch.sh')
    gazebo_script_path = os.path.join(os.path.dirname(__file__), 'gazebo.launch.sh')
    
    px4_process = ExecuteProcess(
        cmd=['/bin/bash', px4_script_path],
        output='screen'
        )
    
    microxrce_process = ExecuteProcess(
        cmd=['/bin/bash', microxrce_script_path],
        output='log'
        )

    spawn_entity = ExecuteProcess(
        cmd=['/bin/bash', gazebo_script_path],
        output='log'
        )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        output='log'
    )

    # Register the SIGINT handler
    signal.signal(signal.SIGINT, sigint_handler)

    ld = LaunchDescription([
        # Launch gazebo environment
        use_sim_time_arg,
        spawn_entity,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[wait_spawn],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_spawn,
                on_exit=[px4_process],
            )
        ),
        px4_process,
        microxrce_process,
        use_groundcontrol,
        ExecuteProcess(cmd=['QGroundControl.AppImage'],
                       condition=IfCondition(LaunchConfiguration('groundcontrol')))
    ])

    return ld
