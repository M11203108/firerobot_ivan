from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),

        Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace'),
            name=LaunchConfiguration('camera_name'),
            executable='realsense2_camera_node',
            parameters=[{
                'enable_color': False,
                'enable_depth': False,
                'enable_infra': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 2,  # linear_interpolation
                'publish_tf': True,
                'angular_velocity_cov': 0.01,
                'linear_accel_cov': 0.01,
            }],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            parameters=[{
                'use_mag': False,
                'gain': 0.05,
                'fixed_frame': 'camera_imu_frame',
                'world_frame': 'enu',
                'orientation_stddev': 0.01,
            }],
            remappings=[('/imu/data_raw', '/camera/camera/imu')],
            output='screen',
            emulate_tty=True,
        ),
    ])
