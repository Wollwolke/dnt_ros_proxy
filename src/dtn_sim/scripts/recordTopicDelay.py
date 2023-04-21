#!/usr/bin/env python3

"""Record the delay of a ROS topic from header timestamp."""

from collections import defaultdict
from time import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped


class DelayRecorder(Node):
    def __init__(self):
        super().__init__("delay_recorder")
        self.records = defaultdict(list)
        self.subscribers = []

        self.create_subscription(
            Image,
            "/detectedImages",
            lambda msg: self.callback(msg, "/detectedImages"),
            10,
        )
        self.create_subscription(
            Temperature,
            "/temperature",
            lambda msg: self.callback(msg, "/temperature"),
            10,
        )
        self.create_subscription(
            RelativeHumidity,
            "/humidity",
            lambda msg: self.callback(msg, "/humidity"),
            10,
        )
        self.create_subscription(
            BatteryState,
            "/status/battery",
            lambda msg: self.callback(msg, "/status/battery"),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/status/tempSensor",
            lambda msg: self.callback(msg, "/status/tempSensor"),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/status/position",
            lambda msg: self.callback(msg, "/status/position"),
            10,
        )

    def __del__(self):
        print("Recorded Delays:")
        for topic, values in self.records.items():
            print(
                f"{topic}: {(sum(values) / len(values) * 1000):.5f}ms - with {len(values)} samples"
            )

    def callback(self, msg, topic):
        now = time()
        try:
            msgTime = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self.records[topic].append(now - msgTime)
        except AttributeError as err:
            print(f"Error reading timestep on topic {topic}: {err}")


def main():
    rclpy.init(args=None)
    node = DelayRecorder()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
