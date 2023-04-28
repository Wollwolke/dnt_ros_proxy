#!/usr/bin/env python3

"""Record the delay of a ROS topic from header timestamp."""

import argparse
import statistics
from collections import defaultdict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped


class DelayRecorder(Node):
    def __init__(self, args):
        super().__init__("delay_recorder")
        self.records = defaultdict(list)
        self.subscribers = []

        self.create_subscription(
            Image,
            f"{args.prefix}/detectedImages",
            lambda msg: self.callback(msg, f"{args.prefix}/detectedImages"),
            10,
        )
        self.create_subscription(
            Temperature,
            f"{args.prefix}/temperature",
            lambda msg: self.callback(msg, f"{args.prefix}/temperature"),
            10,
        )
        self.create_subscription(
            RelativeHumidity,
            f"{args.prefix}/humidity",
            lambda msg: self.callback(msg, f"{args.prefix}/humidity"),
            10,
        )
        self.create_subscription(
            BatteryState,
            f"{args.prefix}/status/battery",
            lambda msg: self.callback(msg, f"{args.prefix}/status/battery"),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            f"{args.prefix}/status/tempSensor",
            lambda msg: self.callback(msg, f"{args.prefix}/status/tempSensor"),
            10,
        )
        self.create_subscription(
            PointStamped,
            f"{args.prefix}/status/position",
            lambda msg: self.callback(msg, f"{args.prefix}/status/position"),
            10,
        )

    def __del__(self):
        print("\nRecorded Delays:")
        for topic, values in self.records.items():
            print(f"{topic} - {len(values)} samples:")
            print(f"mean: {(statistics.mean(values) * 1000):.5f}ms")
            print(f"stdev: {(statistics.stdev(values) * 1000):.5f}ms\n")

    def callback(self, msg, topic):
        now = self.get_clock().now()
        try:
            msgTime = Time.from_msg(msg.header.stamp)
            diff = (now - msgTime).nanoseconds * 1e-9
            print(f"$ {topic} {diff}")
            self.records[topic].append(diff)
        except AttributeError as err:
            print(f"Error reading timestep on topic {topic}: {err}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--prefix",
        help="prefix for topics to subscribe to",
        default="",
    )
    args = parser.parse_args()

    rclpy.init(args=None)
    node = DelayRecorder(args)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
