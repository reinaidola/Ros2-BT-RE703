# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = True

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("ratna_description"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("ratna_description"), "worlds", "empty.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('ratna_description'), 'launch', 'description.launch.py']
    )
    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare("ratna_description"), 'config', 'twist_mux.yaml']
    )

    
    game_sim = Node(
        package='ado_controller_sim',
        executable='ado_controller_sim',
        name = 'ado_controller_sim',
        condition=IfCondition(LaunchConfiguration('use_game_pad')),
        output = 'screen'
    )
    
    teleop_key =  ExecuteProcess(
        cmd=[
            'bash', '-c',
            'gnome-terminal --title=teleop_key -- ros2 run teleop_twist_keyboard teleop_twist_keyboard & '
            'sleep 0.5 && '
            'W=$(xdpyinfo | awk "/dimensions/{print $2}" | cut -d"x" -f1) && '
            'wmctrl -r teleop_key -e 0,$((W-650)),40,600,400'
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_game_pad')),
    )
    
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'publish_joints': 'false',
        }.items()
    )
    
    extended_kalman_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}, 
            ekf_config_path
        ],
        remappings=[("odometry/filtered", "odom")]
    )
    
    twist_mux_launch = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params],
        remappings=[
            ('cmd_vel_out','omni_cont/cmd_vel')
        ]
    )
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'ratna', 
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '1.57',
        ]
    )
    
    execute = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
        output='screen'
    )
    
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('ratna_description'), 'rviz', 'ratna_description.rviz']
    )
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('gazebo_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gazebo_rviz', 
            default_value='true',
            description='Launch RViz alongside Gazebo'
        ),
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),
        DeclareLaunchArgument(
            name='use_game_pad', 
            default_value='false',
            description='Use game pad controller'
        ),
        game_sim,
        # teleop_key,
        description_launch,
        twist_mux_launch,
        extended_kalman_filter,
        spawn_entity_node,
        execute,
        rviz_node
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
