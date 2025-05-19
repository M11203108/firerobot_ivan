#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/lidarttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') #for s2 is 1000000
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    scan_frequency = LaunchConfiguration('scan_frequency', default='10.0') 

    config_dir = get_package_share_directory('motor_base') + '/config'

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': '/dev/lidarttyUSB1',
                         'serial_baudrate': serial_baudrate,
                         'frame_id': 'laser_front',
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode,
                         'scan_frequency': scan_frequency}],
            remappings=[('/scan', '/scan_front')],
            output='screen',
            ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_back',
            parameters=[{'channel_type':channel_type,
                         'serial_port': '/dev/lidarttyUSB0',
                         'serial_baudrate': serial_baudrate,
                         'frame_id': 'laser_back',
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode,
                         'scan_frequency': scan_frequency}],
            remappings=[('/scan', '/scan_back')],
            output='screen',
            ),

        Node(
            package='rplidar_ros',
            executable='angle_filter_node.py',
            name='angle_filter_front',
            parameters=[{
                'min_angle': -80.0,
                'max_angle': 85.0,
                'additional_min_angle': 180.0,
                'additional_max_angle': 360.0
            }],
            remappings=[('/scan', '/scan_front'),
                        ('/scan_filter', '/scan_front_filtered')],
            output='screen'
        ),

        Node(
            package='rplidar_ros',
            executable='angle_filter_node.py',
            name='angle_filter_front',
            parameters=[{
                'min_angle': -80.0,
                'max_angle': 80.0,
                'additional_min_angle': 185.0,
                'additional_max_angle': 360.0
            }],
            remappings=[('/scan', '/scan_back'),
                        ('/scan_filter', '/scan_back_filtered')],
            output='screen'
        ),
        
    ])

