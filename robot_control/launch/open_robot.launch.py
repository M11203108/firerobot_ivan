# File: bringup_all.launch.py   ← 自己取喜歡的檔名，放在 <pkg>/launch/
from pathlib import Path

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():        # ← ← ← 必須叫這個名字
    # 1) 找到各 package 的 launch 目錄
    motor_launch  = str(Path(get_package_share_directory('motor_base'))            / 'launch' / 'robot.launch.py')
    lidar_launch  = str(Path(get_package_share_directory('rplidar_ros'))           / 'launch' / 'rplidar_s2_launch.py')
    merge_launch  = str(Path(get_package_share_directory('ros2_laser_scan_merger'))/ 'launch' / 'merge_2_scan.launch.py')
    imu_launch = str(Path(get_package_share_directory('realsense2_camera'))            / 'launch' / 'imu_launch.py')

    # 2) 建立 IncludeLaunchDescription actions
    robot_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_launch))
    lidar_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch))
    merge_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(merge_launch))
    imu_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_launch))

    # 3) 回傳 LaunchDescription
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        robot_base,
        imu_base,
        lidar_base,
        merge_base
    ])
