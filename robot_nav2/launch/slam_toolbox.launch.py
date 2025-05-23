from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("robot_nav2") + '/config/mapper_params_online_async.yaml',
                {
                    'scan_queue_size': 50,  # 加大佇列容量
                    'use_scan_matching': True,  # 若你想強化 scan 對齊也可以加
                }
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[('odom','odom_combined')]
        )
    ])