#!/usr/bin/env python3

"""Custom Rosbag player, that replaces timestamps with the current RosTime.
May not work for tf tree msgs"""

import argparse
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.serialization import deserialize_message
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
        self.skip = args.skip

    def get_pub(self, topic, type):
        if not topic in self.pubs:
            self.get_logger().info(f"Publishing on topic: {topic}")
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
        while True:
            timeDiff = None
            for topic, msg_type, msg, timestamp in read_messages(self.file, int(self.skip * 1e9)):
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
            if not self.loop:
                return


def read_messages(input_bag: str, skip_nsec: int):
    if 0!= skip_nsec:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        if reader.has_next():
            _, _, timestamp = reader.read_next()
            skip_nsec += timestamp
        del reader

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    if 0 != skip_nsec:
        reader.seek(skip_nsec)

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
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "input", help="Input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "--loop",
        help="Restarts the bag when the end is reached",
        dest="loop",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "--start-offset",
        help="Start the playback this many seconds into the bag file.",
        dest="skip",
        type=int,
        default=0,
    )
    args = parser.parse_args()

    rclpy.init(args=None)
    player = TimelyRosbagPlayer(args)
    player.start()

    try:
        rclpy.spin(player)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
