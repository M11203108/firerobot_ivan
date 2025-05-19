import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_path = get_package_share_directory('zlac8015d_diff_control')
    config_path = os.path.join(pkg_path, 'config', 'motor_set_config.yaml')

    # 啟動 ros2_control_node，不直接傳入 robot_description
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_path],  # 只讀取 YAML 設定，不直接給 robot_description
        output='screen'
    )

    # 5秒後啟動 joint_state_broadcaster
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster']
            )
        ]
    )

    # 5秒後啟動 diff_drive_controller
    diff_drive_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller']
            )
        ]
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
