#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryPathRecorder:
    def __init__(self):
        rospy.init_node("odom_path", anonymous=True)

        self.path = Path()
        self.path.header.frame_id = "map"   # usually "map" or "odom"
        self.pub = rospy.Publisher('/odom_path', Path, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.odom_callback)  # Changed from 0.1 to 1.0

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)

        self.pub.publish(self.path)

if __name__ == "__main__":
    OdometryPathRecorder()
    rospy.spin()
