#!/usr/bin/env python3

import argparse

from rclpy.serialization import deserialize_message, serialize_message
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import rosbag2_py


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
        type_string = typename(topic)
        msg_type = get_message(type_string)
        msg = deserialize_message(data, msg_type)
        yield topic, type_string, msg, timestamp
    del reader


def write_msg(args):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    initializedTopics = set()
    initTime = -1
    cnt = 0
    
    initializedTopics.add("/status/tempSensor")
    writer.create_topic(
        rosbag2_py.TopicMetadata(name="/status/tempSensor", type="diagnostic_msgs/msg/DiagnosticArray", serialization_format="cdr")
    )

    for topic, msg_type, msg, timestamp in read_messages(args.input):
        if not topic in initializedTopics:
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic, type=msg_type, serialization_format="cdr"
                )
            )
            initializedTopics.add(topic)
        if -1 == initTime and "/status/battery" == topic:
            initTime = Time().from_msg(msg.header.stamp).nanoseconds
        if "/status/tempSensor" == topic:
            arr = DiagnosticArray()
            arr.header.stamp = Time(nanoseconds=initTime + cnt * 1e9).to_msg()
            arr.status.append(msg)
            writer.write(topic, serialize_message(arr), timestamp)
            cnt += 1
        else:
            writer.write(topic, serialize_message(msg), timestamp)

    del writer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "output", help="output bag path (folder or filepath) to write to"
    )

    args = parser.parse_args()
    write_msg(args)


if __name__ == "__main__":
    main()
