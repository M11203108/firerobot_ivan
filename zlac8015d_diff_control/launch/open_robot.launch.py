from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zlac8015d_diff_control'),
                'launch',
                'description.launch.py'
            ])
        ])
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zlac8015d_diff_control'),
                'launch',
                'bringup.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        description_launch,
        bringup_launch
    ])
