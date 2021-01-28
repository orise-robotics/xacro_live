#!/usr/bin/env python3
#
# Copyright 2020 Open BR
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize launch description
    launch_description = LaunchDescription()

    # Declare arguments
    launch_description.add_action(
        DeclareLaunchArgument(name='xacro_file', description='Target xacro file.')
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            name='rviz_config',
            description='Rviz config file.',
            default_value=PathJoinSubstitution([
                FindPackageShare('xacro_live'), 'rviz/view_robot.rviz'
            ])
        )
    )

    # Add nodes
    launch_description.add_action(
        Node(
            package='xacro_live',
            executable='xacro_live',
            name='xacro_live',
            output='screen',
            arguments=[LaunchConfiguration('xacro_file')]
        )
    )
    launch_description.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description':
                Command([
                    ExecutableInPackage(package='xacro', executable='xacro'), ' ',
                    LaunchConfiguration('xacro_file')
                ])
            }]
        )
    )
    launch_description.add_action(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    )
    launch_description.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),
    )

    return launch_description
