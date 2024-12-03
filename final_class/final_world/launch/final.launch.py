from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition

import os


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")


    # Declare a launch argument to enable or disable the sensor group
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Enable rviz'
    )
    
    # Use simulation time
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    # Set the path to this package.
    pkg_share = FindPackageShare(package="final_world").find("final_world")
    # launch_file_dir = os.path.join(get_package_share_directory("final_world"), "launch")
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    # Path to the world file
    world_file = os.path.join(pkg_share, "worlds", "final.world")
    


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_file}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
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
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )
    
    target_publisher_cmd = Node(
        package="final_world",
        executable="target_publisher_exe",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )
    
    box_broadcaster_cmd = Node(
        package="final_world",
        executable="box_broadcaster.py",
        output="screen",
    )
    
    # rviz configuration file
    rviz_config_file = os.path.join(pkg_share, "config", "rviz_in_class.rviz")
    # Start rviz and a node that displays the box and the cylinder in rviz
    rviz_group = GroupAction(
        actions=[
            Node(
                package='rviz2', 
                executable='rviz2',
                output='screen',
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
                ),
            Node(
                package='final_world',
                executable='rviz_display.py',
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # Create LaunchDescription object
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(target_publisher_cmd)
    ld.add_action(box_broadcaster_cmd)
    ld.add_action(enable_rviz_arg)
    ld.add_action(rviz_group)

    return ld
