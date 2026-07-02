#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Authors: HyunGyu Kim
# CoAuthor: Chua Wei Jack

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(pkg_share, 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_config_file = os.path.join(pkg_share, 'params', 'multirobot.yaml')
    with open(world_config_file) as file:
        config = yaml.safe_load(file)
        selected_world = config.get('world')
        robots = config['robots']

    world = os.path.join(
        pkg_share,
        'worlds',
        f'{selected_world}_world.world'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models'))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # /clock is a single global topic, so bridge it once rather than per robot.
    clock_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(clock_bridge_cmd)

    for robot in robots:
        namespace = robot['namespace']

        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
            }.items()
        )

        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'x_pose': str(robot['x_pose']),
                'y_pose': str(robot['y_pose']),
            }.items()
        )

        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_turtlebot_cmd)

    return ld
