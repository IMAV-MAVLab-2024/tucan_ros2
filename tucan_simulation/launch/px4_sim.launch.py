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


class WorldPoseFromSdfFrame(Substitution):
    """Substitution that retrieves a frame from SDF."""

    def __init__(self,
                 frame_name: SomeSubstitutionsType,
                 world_name: SomeSubstitutionsType,
                 model_pose: SomeSubstitutionsType,
                 coor_name: str) -> None:
        super().__init__()

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions
        self.__frame_name = normalize_to_list_of_substitutions(frame_name)
        self.__world_name = normalize_to_list_of_substitutions(world_name)
        self.__model_pose = normalize_to_list_of_substitutions(model_pose)
        self.__x = '0.0'
        self.__y = '0.0'
        self.__z = '0.3'
        self.__roll = '0.0'
        self.__pitch = '0.0'
        self.__yaw = '0.0'
        self.__coord_name = coor_name

    @property
    def model_pose(self) -> List[Substitution]:
        """Getter for model pose."""
        return self.__model_pose

    @property
    def frame_name(self) -> List[Substitution]:
        """Getter for frame name."""
        return self.__frame_name

    @property
    def world_name(self) -> List[Substitution]:
        """Getter for world name."""
        return self.__world_name

    def parseCoords(self, strCoords: str,
                    key: Literal['x', 'y', 'z', 'roll', 'pitch', 'yaw'],
                    strSplit: str):
        x, y, z, roll, pitch, yaw = strCoords.split(strSplit)
        if (key == 'x'):
            return str(x)
        if (key == 'y'):
            return str(y)
        if (key == 'z'):
            return str(z)
        if (key == 'roll'):
            return str(roll)
        if (key == 'pitch'):
            return str(pitch)
        if (key == 'yaw'):
            return str(yaw)
        raise Exception('Not able to parse model pose coordinates')

    def perform(self, context: LaunchContext) -> str:
        frame_name_str = perform_substitutions(context, self.frame_name)
        world_name_str = perform_substitutions(context, self.world_name)
        model_pose_str = perform_substitutions(context, self.model_pose)

        # allow manually specified model_pose param to override lookup
        if model_pose_str != '':
            return self.parseCoords(model_pose_str, self.__coord_name, ', ')

        if frame_name_str != '':
            world_sdf_path = os.path.join(
                get_package_share_directory('vehicle_gateway_worlds'),
                'worlds',
                world_name_str + '.sdf')
            # I couldn't get the libsdformat binding to work as expected due
            # to various troubles. Let's simplify and just treat SDF as
            # regular XML and do an XPath query
            sdf_root = ET.parse(world_sdf_path).getroot()
            frame_node = sdf_root.find(f".//frame[@name=\'{frame_name_str}\']")
            if not frame_node:
                raise ValueError(f'Could not find a frame named {frame_name_str}')
            pose_node = frame_node.find('pose')
            pose_str = pose_node.text
            # SDFormat stores poses space-separated, but we need them comma-separated
            return self.parseCoords(pose_str, self.__coord_name, ' ')

        # default a bit above the origin; vehicle will drop to the ground plane
        return self.parseCoords('0, 0, 0.3, 0, 0, 0', self.__coord_name, ', ')


def get_model_pose(frame_name, world_name, model_pose):
    model_pose_x = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='x')

    model_pose_y = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='y')

    model_pose_z = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='z')

    model_pose_roll = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='roll')

    model_pose_pitch = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='pitch')

    model_pose_yaw = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='yaw')
    return [model_pose_x, model_pose_y, model_pose_z,
            model_pose_roll, model_pose_pitch, model_pose_yaw]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    use_groundcontrol = DeclareLaunchArgument('groundcontrol', default_value='true',
                                              choices=['true', 'false'],
                                              description='Start ground control station.')

    world_name = LaunchConfiguration('world_name', default='empty_px4_world')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name (without .sdf)')

    model_pose = LaunchConfiguration('model_pose', default='')
    model_pose_arg = DeclareLaunchArgument('model_pose',
                                           default_value=model_pose,
                                           description='Model pose (x, y, z, roll, pitch, yaw)')

    [model_pose_x, model_pose_y, model_pose_z,
     model_pose_roll, model_pose_pitch, model_pose_yaw] = get_model_pose(
        LaunchConfiguration('frame_name'),
        LaunchConfiguration('world_name'),
        LaunchConfiguration('model_pose'))

    world_pkgs = get_package_share_directory('tucan_simulation')
    gateway_models_dir = get_package_share_directory('tucan_simulation')

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(world_pkgs, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gateway_models_dir, 'models')

    wait_spawn = ExecuteProcess(cmd=["sleep", "5"])
    
    px4_script_path = os.path.join(os.path.dirname(__file__), 'px4.launch.sh')
    microxrce_script_path = os.path.join(os.path.dirname(__file__), 'microxrce.launch.sh')

    px4_process = ExecuteProcess(
        cmd=['/bin/bash', px4_script_path],
        output='screen'
        )
    
    microxrce_process = ExecuteProcess(
        cmd=['/bin/bash', microxrce_script_path],
        output='screen'
        )

    model_name = ["tucan"]

    model_sdf_filename = [
        gateway_models_dir,
        '/tucan',
        '/model.sdf']


    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', model_sdf_filename,
                   '-name', "tucan",
                   '-allow_renaming', 'true',
                   '-x', model_pose_x,
                   '-y', model_pose_y,
                   '-z', model_pose_z,
                   '-R', model_pose_roll,
                   '-P', model_pose_pitch,
                   '-Y', model_pose_yaw])

    model_name_env_var = SetEnvironmentVariable('PX4_GZ_MODEL_NAME', model_name)

    autostart_env_var = SetEnvironmentVariable(
        'PX4_SYS_AUTOSTART',
        'autostart_magic_number')

    os.environ['PX4_GZ_WORLD'] = ""

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
        world_name_arg,
        model_pose_arg,
        spawn_entity,
        model_name_env_var,
        autostart_env_var,
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

    if len(command_output) == 0:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), '.sdf'])]
        ))
    else:
        print('Another gz instance is running, it will only try to spawn the model in gz.')

    return ld
