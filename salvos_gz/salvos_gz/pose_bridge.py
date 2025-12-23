#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import gz.transport15 as gz
from gz.msgs12.pose_v_pb2 import Pose_V


def import_gz():
    transport = None
    for ver in (15, 14, 13):
        try:
            transport = __import__(f"gz.transport{ver}", fromlist=["Node"])
            break
        except ImportError:
            pass
    if transport is None:
        raise ImportError("Install python3-gz-transportXX")

    PoseV = None
    for ver in (12, 11, 10):
        try:
            PoseV = getattr(__import__(f"gz.msgs{ver}.pose_v_pb2", fromlist=["Pose_V"]), "Pose_V")
            break
        except ImportError:
            pass
    if PoseV is None:
        # last fallback
        PoseV = getattr(__import__("gz.msgs.pose_v_pb2", fromlist=["Pose_V"]), "Pose_V")

    return transport, PoseV

#xd, Pose_V = import_gz()


class PoseBridge(Node):

    def __init__(self, model="hermes", link="frame"):
        super().__init__('pose_bridge')

        # Target names
        self.target = f"{model}::{link}"
        self.topic_gz = f"/model/{model}/pose"
        self.topic_ros = f"/{model}/{link}/pose"

        # ROS publisher
        self.pub = self.create_publisher(PoseStamped, self.topic_ros, 10)
        self.get_logger().info(f"Publishing {self.target} pose → {self.topic_ros}")

        self.gz_node = gz.Node()
        self.gz_node.subscribe(self.topic_gz, Pose_V, self._on_pose)

        # Spin gz in a background thread
        threading.Thread(target=self.gz_node.spin, daemon=True).start()
        self.get_logger().info(f"Subscribed to {self.topic_gz}")

    def _on_pose(self, msg):
        """Callback from Gazebo: find hermes::frame and publish to ROS."""
        for p in msg.pose:
            if p.name == self.target:
                ros_msg = PoseStamped()
                ros_msg.header.stamp = self.get_clock().now().to_msg()
                ros_msg.header.frame_id = "world"
                ros_msg.pose.position.x = p.position.x
                ros_msg.pose.position.y = p.position.y
                ros_msg.pose.position.z = p.position.z
                ros_msg.pose.orientation.x = p.orientation.x
                ros_msg.pose.orientation.y = p.orientation.y
                ros_msg.pose.orientation.z = p.orientation.z
                ros_msg.pose.orientation.w = p.orientation.w
                self.pub.publish(ros_msg)
                break


def main():
    rclpy.init()
    node = PoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
