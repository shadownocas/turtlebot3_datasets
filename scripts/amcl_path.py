#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class AMCLPathRecorder:
    def __init__(self):
        rospy.init_node("publish_amcl_path")
        self.path = Path()
        self.path.header.frame_id = "map"
        self.pub = rospy.Publisher("/amcl_path", Path, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
    
    def pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.pub.publish(self.path)

if __name__ == "__main__":
    AMCLPathRecorder()
    rospy.spin()
