import os
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0")
    y_pose = LaunchConfiguration("y_pose", default="0")
    yaw_pose = LaunchConfiguration("yaw_pose", default="-1.5707")
    rviz = LaunchConfiguration("rviz", default="false")
    broadcast = LaunchConfiguration("broadcast", default="false")

    # Set the path to this package.
    pkg_share = FindPackageShare(package="final_world").find("final_world")

    # Set the path to the world file
    world_file_name = "final_in_class.world"
    world_path = os.path.join(pkg_share, "worlds", world_file_name)

    user_config_path = os.path.join(pkg_share, "config", "sensors.yaml")

    if not os.path.exists(user_config_path):
        rclpy.logging.get_logger("Launch File").fatal(
            f"Sensor configuration 'sensors.yaml' not found in config directory: {user_config_path}."
        )
        exit()

    rviz_config_file = os.path.join(pkg_share, "config", "frame.rviz")

    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": world_path,
        }.items(),
    )

    # part_publisher_node
    target_publisher_node = Node(
        package="final_world",
        executable="target_publisher_exe",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Environment Startup
    environment_startup = Node(
        package="final_world",
        executable="environment_startup_node.py",
        output="screen",
        parameters=[
            {"user_config_path": user_config_path},
            {"use_sim_time": True},
        ],
    )

    static_transform_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # r
            "0",  # p
            "0",  # y
            "world",  # frame_id
            "odom",  # child_frame_id
        ],
    )

    # start rviz only if the argument is set to true
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    launch_file_dir = os.path.join(
        get_package_share_directory("final_world"), "launch"
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw_pose": yaw_pose,
        }.items(),
    )

    nodes_to_start = [
        gazebo,
        environment_startup,
        # part_spawner_cmd,
        # robot_target_cmd,
        # start_aruco_detection_node_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        # turtlebot3_navigation2_cmd,
        # sensor_tf_broadcaster,
        static_transform_cmd,
        # broadcast_cmd,
        rviz_cmd,
        target_publisher_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sensor_config",
            default_value="sensors",
            description="name of user configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Whether to start RViz",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "broadcast",
            default_value="false",
            description="Whether to broadcast the new frame",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
