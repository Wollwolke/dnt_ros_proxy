#!/usr/bin/env python3

"""Record the movement of the robot in 0.2s steps to a file."""

import argparse

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MovementExtractor(Node):
    def __init__(self, args):
        super().__init__("movement_extractor")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.file = open(args.output, "w")
        self.timer = self.create_timer(0.2, self.on_timer)

    def __del__(self):
        self.file.close()

    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform()
            result = f"{transform.transform.translation.x} {transform.transform.translation.y}\n"
            self.file.write(result)
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform map to base_footprint: {ex}",
                throttle_duration_sec=3,
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="output file path")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = MovementExtractor(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
