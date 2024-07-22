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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext, SomeSubstitutionsType, Substitution
from launch.utilities import perform_substitutions
from typing import List, Literal
import os
from launch.substitutions import LaunchConfiguration, PythonExpression
from subprocess import Popen, PIPE
from shlex import split

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    use_groundcontrol = DeclareLaunchArgument('groundcontrol', default_value='true',
                                              choices=['true', 'false'],
                                              description='Start ground control station.')

    world_pkgs = get_package_share_directory('tucan_simulation')
    gateway_models_dir = get_package_share_directory('tucan_simulation')

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
        output='screen'
        )

    spawn_entity = ExecuteProcess(
        cmd=['/bin/bash', gazebo_script_path],
        output='screen'
        )

    p1 = Popen(split("gz topic -l"), stdout=PIPE)
    p2 = Popen(split("grep -m 1 -e '/world/.*/clock'"), stdin=p1.stdout, stdout=PIPE)
    p3 = Popen(split(r"sed 's/\/world\///g; s/\/clock//g'"), stdin=p2.stdout, stdout=PIPE)
    command_output = p3.stdout.read().decode('utf-8')

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

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
