#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dtn_robot_interfaces.msg import PointArray
from dtn_robot_interfaces.srv import FollowWaypoints


class MyClient(Node):
    def __init__(self):
        super().__init__("sendWaypointsClient")

        self.declare_parameter("wpFile", "")
        filePath = self.get_parameter("wpFile").value
        if filePath:
            self.wpFilePath = filePath
        else:
            print("Missing Argument, Run Usage: --ros-args -p wpFile:=<path/to/wpFile>")
            exit(1)

        self.cli = self.create_client(FollowWaypoints, "follow_waypoints")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = FollowWaypoints.Request()

    def send_request(self):
        pointList = PointArray()
        pointList.header.frame_id = "map"
        with open(self.wpFilePath) as f:
            for line in f.readlines():
                x, y = line.split(" ")
                point = Point()
                point.x = float(x)
                point.y = float(y)

                pointList.points.append(point)
        self.req.waypoints = pointList
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = MyClient()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error("Service call failed %r" % (e,))
            else:
                client.get_logger().info(
                    "Request Accepted: %d" % (response.request_accepted)
                )
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
