#!/usr/bin/env python3

import os
import rospy
from nav_msgs.msg import Odometry


class OdomToTumRecorder:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/vo/odom")
        self.output_file = rospy.get_param("~tum_output_file", "vo_tum.txt")
        self.append = rospy.get_param("~append", False)
        self.flush_interval = max(1, int(rospy.get_param("~flush_interval", 100)))

        mode = "a" if self.append else "w"
        self.lines_since_flush = 0

        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        self.fp = open(self.output_file, mode, buffering=1)
        rospy.loginfo("odom_to_tum: writing TUM poses to %s (append=%s, flush_interval=%d)",
                      self.output_file, str(self.append).lower(), self.flush_interval)

        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=200)
        rospy.on_shutdown(self.on_shutdown)

    def odom_callback(self, msg):
        stamp = msg.header.stamp.to_sec()
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

    def on_shutdown(self):
        try:
            if self.fp:
                self.fp.flush()
                self.fp.close()
        except Exception:
            pass


def main():
    rospy.init_node("odom_to_tum")
    OdomToTumRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
