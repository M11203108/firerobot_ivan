from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_base',
            executable='imu_to_base_frame.py',
            name='imu_to_base_frame',
            output='screen'
        )
    ])
