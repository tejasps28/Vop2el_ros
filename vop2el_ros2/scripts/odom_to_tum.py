#!/usr/bin/env python3

import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToTumRecorder(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_tum")

        self.declare_parameter("odom_topic", "/vo/odom")
        self.declare_parameter("tum_output_file", "vo_tum.txt")
        self.declare_parameter("append", False)
        self.declare_parameter("flush_interval", 100)

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.output_file = (
            self.get_parameter("tum_output_file").get_parameter_value().string_value
        )
        self.append = self.get_parameter("append").get_parameter_value().bool_value
        self.flush_interval = max(
            1, self.get_parameter("flush_interval").get_parameter_value().integer_value
        )

        mode = "a" if self.append else "w"
        self.lines_since_flush = 0

        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        self.fp = open(self.output_file, mode, buffering=1, encoding="utf-8")
        self.get_logger().info(
            "odom_to_tum: writing TUM poses to %s (append=%s, flush_interval=%d)"
            % (self.output_file, str(self.append).lower(), self.flush_interval)
        )

        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 200
        )

    def odom_callback(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        line = "{:.9f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
            stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w
        )
        self.fp.write(line)
        self.lines_since_flush += 1
        if self.lines_since_flush >= self.flush_interval:
            self.fp.flush()
            self.lines_since_flush = 0

    def close(self) -> None:
        try:
            if self.fp:
                self.fp.flush()
                self.fp.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = OdomToTumRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
