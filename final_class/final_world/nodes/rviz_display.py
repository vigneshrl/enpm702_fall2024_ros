#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        self.timer = self.create_timer(1.0, self.publish_markers)  # Publish markers at 1 Hz

    def publish_markers(self):
        # Cube Marker
        cube_marker = Marker()
        cube_marker.header.frame_id = "odom"
        cube_marker.header.stamp = self.get_clock().now().to_msg()
        cube_marker.ns = "shapes"
        cube_marker.id = 0
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = 5.0
        cube_marker.pose.position.y = 5.0
        cube_marker.pose.position.z = 0.25
        cube_marker.pose.orientation.x = 0.0
        cube_marker.pose.orientation.y = 0.0
        cube_marker.pose.orientation.z = 0.0
        cube_marker.pose.orientation.w = 1.0
        cube_marker.scale.x = 0.2
        cube_marker.scale.y = 0.2
        cube_marker.scale.z = 0.2
        cube_marker.color.r = 1.0
        cube_marker.color.g = 0.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 0.5  # Semi-transparent

        # Cylinder Marker
        cylinder_marker = Marker()
        cylinder_marker.header.frame_id = "odom"
        cylinder_marker.header.stamp = self.get_clock().now().to_msg()
        cylinder_marker.ns = "shapes"
        cylinder_marker.id = 1
        cylinder_marker.type = Marker.CYLINDER
        cylinder_marker.action = Marker.ADD
        cylinder_marker.pose.position.x = 9.0
        cylinder_marker.pose.position.y = 9.0
        cylinder_marker.pose.position.z = 0.5
        cylinder_marker.pose.orientation.x = 0.0
        cylinder_marker.pose.orientation.y = 0.0
        cylinder_marker.pose.orientation.z = 0.0
        cylinder_marker.pose.orientation.w = 1.0
        cylinder_marker.scale.x = 1.0
        cylinder_marker.scale.y = 1.0
        cylinder_marker.scale.z = 1.0
        cylinder_marker.color.r = 1.0
        cylinder_marker.color.g = 1.0
        cylinder_marker.color.b = 0.0
        cylinder_marker.color.a = 0.5  # Semi-transparent

        # Publish the markers
        self.publisher.publish(cube_marker)
        self.publisher.publish(cylinder_marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
