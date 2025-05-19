import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 獲取套件內的 Xacro 檔案路徑
    pkg_path = get_package_share_directory('zlac8015d_diff_control')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    # 轉換 Xacro 為 URDF
    urdf_doc = xacro.process_file(xacro_file)
    with open(urdf_file, 'w') as f:
        f.write(urdf_doc.toxml())

    # 啟動 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_doc.toxml()}],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        robot_state_publisher_node,
    ])
