from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir = get_package_share_directory('motor_base')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("motor_base"), "description", "robot.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}



    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("motor_base"),
            "config",
            "ros2_control.yaml",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel", "cmd_vel")
        # ]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",  # 確保這與 `ros2_control.yaml` 的名稱一致
            "--controller-manager",
            "/controller_manager",
        ],
    )

    static_tf_pub_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.48714", "0.1385", "0.975", "0", "0", "0", "base_link", "camera_link"],
    )

    static_tf_pub_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "-1.5708", "1.5708", "camera_imu_optical_frame", "imu_link"],
    )

    static_tf_lidar_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.435", "-0.3285", "0.2", "0", "0", "0", "base_link", "laser_front"],
    )

    static_tf_lidar_back = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.435", "0.3285", "0.2", "3.1415926", "0", "0", "base_link", "laser_back"],
    )

    robot_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[PathJoinSubstitution(
            [FindPackageShare("motor_base"), "config", "ekf_wh.yaml"]
        )],
    )

    nodes = [
        control_node,
        robot_controller_spawner,
        robot_state_publisher,
        joint_state_publisher_node,
        robot_ekf,
        static_tf_pub_camera,
        # static_tf_pub_imu,
        static_tf_lidar_front,
        static_tf_lidar_back,
    ]

    return LaunchDescription(declared_arguments + nodes)
