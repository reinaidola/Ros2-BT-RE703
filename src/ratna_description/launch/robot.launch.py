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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('ratna_description'), 'launch', 'description.launch.py']
    )
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path)
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("ratna_description"), "config", "ekf.yaml"]
    )
    ekf_launch =Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path
        ],
        remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
    )
    
    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare('ratna_description'), 'config', 'twist_mux.yaml']
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
    
    imu_filter_madgwick = Node(
        condition=IfCondition(LaunchConfiguration("madgwick")),
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='madgwick_filter_node',
        output='screen',
        parameters=[
            {'orientation_stddev' : LaunchConfiguration('orientation_stddev')}
        ]
    )
    robot_controller =Node(
        package='robot_controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        remappings=[
            ('cmd_vel_joy','vel_to_smooth')
        ]
    )
    movement_smoother_launch = Node(
        package='movement_smoother',
        executable='movement_smoother',
        name='movement_smoother',
        output='screen'
    )
    
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a3_launch.py']
            )
        ),launch_arguments={
            'frame_id': 'laser',
            'serial_port': '/dev/rplidar'
        }
    )
    
    micro_ros_launch = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev', LaunchConfiguration("base_serial_port"),
            '--baudrate','115200'
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/serial/by-id/usb-Teensyduino_USB_Serial_15356540-if00',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        DeclareLaunchArgument(
            name='madgwick',
            default_value='false',
            description='Use madgwick to fuse imu and magnetometer'
        ),

        DeclareLaunchArgument(
            name='orientation_stddev',
            default_value='0.003162278',
            description='Madgwick orientation stddev'
        ),
        
        sensor_launch,
        
        micro_ros_launch,
        
        imu_filter_madgwick,
        
        launch_description,
        
        robot_controller,
        
        movement_smoother_launch,
        
        twist_mux_launch,

        ekf_launch,
    ])
