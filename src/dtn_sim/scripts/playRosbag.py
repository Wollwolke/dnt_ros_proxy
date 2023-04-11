#!/usr/bin/env python3

"""Custom Rosbag player, that replaces timestamps with the current RosTime."""

import argparse
import threading

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
import tf2_msgs

import rosbag2_py


class TimelyRosbagPlayer(Node):
    def __init__(self, args):
        super().__init__("rosbag_player")
        self.pubs = {}
        self.file = args.input
        self.loop = args.loop

    def get_pub(self, topic, type):
        if not topic in self.pubs:
            if "/tf_static" == topic:
                self.pubs[topic] = self.create_publisher(
                    type,
                    topic,
                    qos_profile=QoSProfile(
                        depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
                    ),
                )
            else:
                self.pubs[topic] = self.create_publisher(type, topic, 10)
        return self.pubs[topic]

    def start(self):
        self.th = threading.Thread(target=self.play)
        self.th.start()

    def play(self):
        while self.loop:
            timeDiff = None
            for topic, msg_type, msg, timestamp in read_messages(self.file):
                pub = self.get_pub(topic, msg_type)

                if None == timeDiff:
                    timeDiff = self.get_clock().now().nanoseconds - timestamp
                else:
                    self.get_clock().sleep_until(
                        Time(
                            nanoseconds=timestamp + timeDiff,
                            clock_type=ClockType.ROS_TIME,
                        ),
                        context=self.context,
                    )

                if not isinstance(msg, tf2_msgs.msg._tf_message.TFMessage):
                    # non tf msgs
                    try:
                        msg.header.stamp = self.get_clock().now().to_msg()
                    except AttributeError:
                        self.get_logger().info(
                            f"Message on topic {topic} has no timestamp attribute.",
                            throttle_duration_sec=10,
                        )
                else:
                    # tf msgs, have a different time reference?
                    pass

                if not rclpy.ok():
                    return
                pub.publish(msg)


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg_type, msg, timestamp
    del reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "--loop",
        help="restarts the bag when the end is reached",
        dest="loop",
        default=False,
        action="store_true",
    )
    args = parser.parse_args()

    rclpy.init(args=None)
    player = TimelyRosbagPlayer(args)
    player.start()

    try:
        rclpy.spin(player)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
