#!/usr/bin/env python3
# Copyright 2023 ES-SLAM Project
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    scan_topicname_arg = DeclareLaunchArgument(
        'scan_topicname',
        default_value='scan',
        description='Topic name for laser scan'
    )
    
    map_topicname_arg = DeclareLaunchArgument(
        'map_topicname',
        default_value='map',
        description='Topic name for map output'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        # default_value='laser2',
        default_value='base_footprint',
        description='Base frame ID'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )
    
    initial_map_size_arg = DeclareLaunchArgument(
        'initial_map_size',
        default_value='10',
        description='Initial map size'
    )
    
    err_arg = DeclareLaunchArgument(
        'err',
        default_value='-999999999',
        description='Error parameter'
    )
    
    map_rate_arg = DeclareLaunchArgument(
        'map_rate',
        default_value='50.0',
        description='Map update rate'
    )
    
    t2_arg = DeclareLaunchArgument(
        't2',
        default_value='500',
        description='T2 parameter'
    )
    
    ganm_arg = DeclareLaunchArgument(
        'ganm',
        default_value='100',
        description='GANM parameter'
    )
    
    transformed_frame_arg = DeclareLaunchArgument(
        'transformed_frame',
        default_value='laser2',
        description='Transformed frame ID'
    )

    # Static transform publishers
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_static_tf',
        arguments=['0.25', '0.0', '0.0', '0.0', '0.0', '0.0', 
                  LaunchConfiguration('base_frame'), 'laser']
    )
    
    base_to_laser2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser2_static_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                  LaunchConfiguration('base_frame'), LaunchConfiguration('transformed_frame')]
    )

    # ES-SLAM node
    es_slam_node = Node(
        package='es_slam',
        executable='es_slam_node',
        name='es_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': LaunchConfiguration('base_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'initial_map_size': LaunchConfiguration('initial_map_size'),
            'err': LaunchConfiguration('err'),
            'map_rate': LaunchConfiguration('map_rate'),
            't2': LaunchConfiguration('t2'),
            'ganm': LaunchConfiguration('ganm'),
            'transformed_frame': LaunchConfiguration('transformed_frame'),
            # Log-odds parameters
            'use_logodds': False,
            'logodds_occ': 0.2,
            'logodds_free': -0.1,
            'logodds_max': 5.0,
            'logodds_min': -5.0,
            'logodds_thresh_occ': 0.5,
            'logodds_thresh_free': -0.5,
            # Resampling parameters
            'use_resampling': True,
            'resample_distance_threshold': 0.05,
            'resample_length_threshold': 0.25,
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topicname')),
            ('map', LaunchConfiguration('map_topicname')),
        ]
    )

    # RViz2 node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('es_slam'),
        'rviz',
        'es_slam.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        scan_topicname_arg,
        map_topicname_arg,
        base_frame_arg,
        map_frame_arg,
        initial_map_size_arg,
        err_arg,
        map_rate_arg,
        t2_arg,
        ganm_arg,
        transformed_frame_arg,
        
        # Nodes
        # base_to_laser_tf,
        base_to_laser2_tf,
        es_slam_node,
        rviz_node,
    ])