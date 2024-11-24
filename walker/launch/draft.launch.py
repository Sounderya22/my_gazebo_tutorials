#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Sounderya Varagur Venugopal

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package paths
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_walker = get_package_share_directory('walker')

    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # World file
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_dqn_stage2.world'
    )

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Launch TurtleBot Controller Node
    turtlebot_controller_node = Node(
        package='walker',
        executable='obstacle_detection',
        name='turtlebot_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Rosbag recording
    bag_directory = os.path.join(os.getcwd(), 'results')
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Flag to enable or disable bag recording'
    )

    def conditionally_add_bag_recording(context):
        enable_recording = LaunchConfiguration('enable_recording').perform(context)
        if enable_recording.lower() == 'true':
            return [
                ExecuteProcess(
                    cmd=['mkdir', '-p', bag_directory],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', os.path.join(bag_directory, f'recording_{timestamp}')],
                    output='screen'
                ),
            ]
        return []

    # Launch Description
    ld = LaunchDescription()

    # Add actions to launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(turtlebot_controller_node)
    ld.add_action(enable_recording_arg)
    ld.add_action(OpaqueFunction(function=conditionally_add_bag_recording))

    return ld
