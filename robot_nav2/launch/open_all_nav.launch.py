# robot_nav2/launch/bringup_three.launch.py
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_nav2')
    pkg_robot = get_package_share_directory('motor_base')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_robot) / 'launch/open_robot.launch.py')
        ),
    )

    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_share) / 'launch/localization.launch.py')
        ),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_share) / 'launch/navigation.launch.py')
        ),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_share) / 'launch/rviz.launch.py')
        )
    )


    cmd_vel_bridge = Node(
        package='motor_base',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[]
    )



    return LaunchDescription([
        robot_launch,

        TimerAction(period=0.5, actions=[cmd_vel_bridge]),
        TimerAction(period=0.5, actions=[rviz_launch]),
        TimerAction(period=1.5, actions=[loc_launch]),
        TimerAction(period=3.5, actions=[nav_launch]),
    ])
