#!/usr/bin/env python3

import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    rb2_param_dir = LaunchConfiguration(
        'rb2_param_dir',
        default=os.path.join(
            get_package_share_directory('risbot2_bringup'),
            'param',
            'risbot2.yaml'))

    hokuyo_launch_file = LaunchConfiguration(
        'hokuyo_launch_file',
        default=os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py'))
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py')
        ]),
    )
   
    t265_multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_multi_camera_launch.py') 
        ]),
        launch_arguments={'enable_pose_jumping': 'false'}.items(),
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Transform giữa T265 và robot
    # t265_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0.45", "0", "0.9", "0", "0", "0", "base_footprint", "odom_frame"]
    # )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'rb2_param_dir',
            default_value=rb2_param_dir,
            description='Full path to risbot2 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/risbot2_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hokuyo_launch_file]),
        ),

        t265_multi_launch,
        ekf_launch,
        # t265_tf,

        Node(
            package='risbot2_node',
            executable='risbot2_node.py',
            output='screen'),
    ])
