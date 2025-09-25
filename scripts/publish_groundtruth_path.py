#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class GroundtruthPath:
    def __init__(self):
        rospy.init_node("publish_groundtruth_path")

        self.pub = rospy.Publisher("/groundtruth_path", Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "map"  # Use odom as fixed frame

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Timer at 20 Hz
        rospy.Timer(rospy.Duration(0.05), self.timer_cb)

        rospy.loginfo("Groundtruth path publisher started, waiting for TF...")

    def timer_cb(self, event):
        try:
            # Transform mocap_laser_link → map
            if self.tf_buffer.can_transform("map", "mocap_laser_link", rospy.Time(0), rospy.Duration(1.0)):
                trans = self.tf_buffer.lookup_transform("map", "mocap_laser_link", rospy.Time(0))

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = trans.transform.translation.z
                pose.pose.orientation = trans.transform.rotation

                # Append to path
                self.path.header.stamp = rospy.Time.now()
                self.path.poses.append(pose)
                self.pub.publish(self.path)
            else:
                rospy.logwarn_throttle(2.0, "TF mocap_laser_link → odom not available yet")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"Groundtruth transform failed: {e}")

if __name__ == "__main__":
    GroundtruthPath()
    rospy.spin()
