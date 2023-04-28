#!/usr/bin/env python3

"""Publishes position of a node in core emulator in ROS"""

import argparse
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PointStamped


class CorePosition(Node):
    def __init__(self, nodeName):
        super().__init__("core_pos_publisher")
        self.nodeName = nodeName
        self.pos = (0, 0)
        self.lock = threading.Lock()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(PointStamped, "/status/position", 10)

        self.updateTimer = self.create_timer(0.3, self.updatePos)
        self.sendPosTimer = self.create_timer(1.0, self.sendPos)


    def getNodePos(self):
        corePos = subprocess.run(["core-pos", self.nodeName], stdout=subprocess.PIPE)
        try:
            corePosOutput = corePos.stdout.decode("utf-8").split("\n")
            x = float(corePosOutput[1].split()[1][:-1:])
            y = float(corePosOutput[2].split()[1][:-1:])
            with self.lock:
                self.pos = (x, y)
        except IndexError as _:
            raise ValueError(f"Node {self.nodeName} doesn't exist in current simulation.")


    def updatePos(self):
        self.getNodePos()

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.pos[0]
        t.transform.translation.y = self.pos[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    
    def sendPos(self):
        point = PointStamped()

        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "map"

        with self.lock:
            point.point.x = self.pos[0]
            point.point.y = self.pos[1]
        point.point.z = 0.0
        
        self.pub.publish(point)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "node", help="Name of the node in core emulator"
    )
    args, _ = parser.parse_known_args()

    rclpy.init(args=None)
    node = CorePosition(args.node)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
