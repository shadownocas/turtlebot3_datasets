#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

class EstimatedPath:
    def __init__(self):
        rospy.init_node("estimated_path")

        self.pub = rospy.Publisher("/estimated_path", Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "map"  # Publish everything in map frame

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

    def odom_cb(self, msg: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        try:
            # Wait for the transform: base_footprint → mocap_laser_link
            if self.tf_buffer.can_transform("mocap_laser_link", "base_footprint", rospy.Time(0), rospy.Duration(1.0)):
                # Transform EKF pose to mocap frame
                pose_in_mocap = self.tf_buffer.transform(pose_stamped, "mocap_laser_link")

                # Then transform to map frame
                if self.tf_buffer.can_transform("map", "mocap_laser_link", rospy.Time(0), rospy.Duration(1.0)):
                    pose_in_map = self.tf_buffer.transform(pose_in_mocap, "map")

                    pose_in_map.header.stamp = rospy.Time.now()
                    pose_in_map.header.frame_id = "map"

                    self.path.header.stamp = rospy.Time.now()
                    self.path.poses.append(pose_in_map)
                    self.pub.publish(self.path)
                else:
                    rospy.logwarn_throttle(2.0, "TF mocap_laser_link → map not available yet")
            else:
                rospy.logwarn_throttle(2.0, "TF base_footprint → mocap_laser_link not available yet")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"Estimated transform failed: {e}")

if __name__ == "__main__":
    EstimatedPath()
    rospy.spin()
