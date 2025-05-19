#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 後面 Lidar 的配置
    channel_type_1 = LaunchConfiguration('channel_type_1', default='serial')
    serial_port_1 = LaunchConfiguration('serial_port_1', default='/dev/lidarttyUSB0')
    serial_baudrate_1 = LaunchConfiguration('serial_baudrate_1', default='1000000') 
    frame_id_1 = LaunchConfiguration('frame_id_1', default='laser_1')
    inverted_1 = LaunchConfiguration('inverted_1', default='false')
    angle_compensate_1 = LaunchConfiguration('angle_compensate_1', default='true')
    scan_mode_1 = LaunchConfiguration('scan_mode_1', default='Standard')
    scan_frequency_1 = LaunchConfiguration('scan_frequency_1', default='10.0')

    # 前面 Lidar 的配置
    channel_type_2 = LaunchConfiguration('channel_type_2', default='serial')
    serial_port_2 = LaunchConfiguration('serial_port_2', default='/dev/lidarttyUSB1')
    serial_baudrate_2 = LaunchConfiguration('serial_baudrate_2', default='1000000')
    frame_id_2 = LaunchConfiguration('frame_id_2', default='laser_2')
    inverted_2 = LaunchConfiguration('inverted_2', default='false')
    angle_compensate_2 = LaunchConfiguration('angle_compensate_2', default='true')
    scan_mode_2 = LaunchConfiguration('scan_mode_2', default='Standard')
    scan_frequency_2 = LaunchConfiguration('scan_frequency_2', default='10.0')

    return LaunchDescription([
        # 後面 Lidar 的 Launch Arguments
        # DeclareLaunchArgument('channel_type_1', default_value=channel_type_1, description='Channel type for Lidar 1'),
        # DeclareLaunchArgument('serial_port_1', default_value=serial_port_1, description='Serial port for Lidar 1'),
        # DeclareLaunchArgument('serial_baudrate_1', default_value=serial_baudrate_1, description='Baudrate for Lidar 1'),
        # DeclareLaunchArgument('frame_id_1', default_value=frame_id_1, description='Frame ID for Lidar 1'),
        # DeclareLaunchArgument('inverted_1', default_value=inverted_1, description='Inverted scan data for Lidar 1'),
        # DeclareLaunchArgument('angle_compensate_1', default_value=angle_compensate_1, description='Angle compensation for Lidar 1'),
        # DeclareLaunchArgument('scan_mode_1', default_value=scan_mode_1, description='Scan mode for Lidar 1'),
        # DeclareLaunchArgument('scan_frequency_1', default_value=scan_frequency_1, description='Scan frequency for Lidar 1'),

        # 前面 Lidar 的 Launch Arguments
        DeclareLaunchArgument('channel_type_2', default_value=channel_type_2, description='Channel type for Lidar 2'),
        DeclareLaunchArgument('serial_port_2', default_value=serial_port_2, description='Serial port for Lidar 2'),
        DeclareLaunchArgument('serial_baudrate_2', default_value=serial_baudrate_2, description='Baudrate for Lidar 2'),
        DeclareLaunchArgument('frame_id_2', default_value=frame_id_2, description='Frame ID for Lidar 2'),
        DeclareLaunchArgument('inverted_2', default_value=inverted_2, description='Inverted scan data for Lidar 2'),
        DeclareLaunchArgument('angle_compensate_2', default_value=angle_compensate_2, description='Angle compensation for Lidar 2'),
        DeclareLaunchArgument('scan_mode_2', default_value=scan_mode_2, description='Scan mode for Lidar 2'),
        DeclareLaunchArgument('scan_frequency_2', default_value=scan_frequency_2, description='Scan frequency for Lidar 2'),
        

        # 第一顆 Lidar 節點
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar_node_1',
        #     parameters=[{'channel_type': channel_type_1,
        #                  'serial_port': serial_port_1,
        #                  'serial_baudrate': serial_baudrate_1,
        #                  'frame_id': frame_id_1,
        #                  'inverted': inverted_1,
        #                  'angle_compensate': angle_compensate_1,
        #                  'scan_mode': scan_mode_1,
        #                  'scan_frequency': scan_frequency_1}],
        #     remappings=[('/scan', '/lidar_1/scan')],
        #     output='screen'),

        # 第二顆 Lidar 節點
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_2',
            parameters=[{'channel_type': channel_type_2,
                         'serial_port': serial_port_2,
                         'serial_baudrate': serial_baudrate_2,
                         'frame_id': frame_id_2,
                         'inverted': inverted_2,
                         'angle_compensate': angle_compensate_2,
                         'scan_mode': scan_mode_2,
                         'scan_frequency': scan_frequency_2}],
            remappings=[('/scan', '/lidar_2/scan')],
            output='screen'),
        
        Node(
            package='rplidar_ros',
            executable='angle_filter_node.py',
            name='angle_filter_node',
            parameters=[{'min_angle': -80.0, #-90
                            'max_angle': 0.0, #90
                            'additional_min_angle': 190.0,
                            'additional_max_angle': 360.0}],
            remappings=[('/scan', '/lidar_2/scan'),
                        ('/filtered_scan', '/laser1')],
            output='screen'
        )


    ])