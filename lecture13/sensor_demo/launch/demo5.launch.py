from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():
    ld = LaunchDescription()

    # Grouped nodes
    sensor_group = GroupAction([
        Node(package='sensor_demo', executable='lidar_demo'),
        Node(package='sensor_demo', executable='camera_demo')
    ])

    ld.add_action(sensor_group)
    return ld