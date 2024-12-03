#!/usr/bin/env python3

import rclpy
from final_world.tf_broadcaster import TFBroadcaster
from final_world.utilities import pose_info


def main():
    rclpy.init()

    objects_tf_broadcaster = TFBroadcaster("box_broadcaster")

    objects_tf_broadcaster.generate_transform(
        "odom", "box_frame", pose_info([5, 5, 0.0], [0, 0, 0])
    )
    objects_tf_broadcaster.send_transforms()

    try:
        rclpy.spin(objects_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
