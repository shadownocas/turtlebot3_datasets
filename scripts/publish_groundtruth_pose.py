#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

class GroundtruthPublisher:
    def __init__(self):
        rospy.init_node("publish_groundtruth_pose", anonymous=True)

        # Publisher for groundtruth pose (PoseWithCovarianceStamped)
        self.pub = rospy.Publisher('/groundtruth_pose', PoseWithCovarianceStamped, queue_size=10)

        # TF listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publish at 1 Hz (can increase if mocap supports it)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_groundtruth_pose)

    def publish_groundtruth_pose(self, event):
        try:
            # lookup latest transform (map <- mocap_laser_link)
            # Use a short timeout so we don't hang the timer
            trans = self.tf_buffer.lookup_transform("map", "mocap_laser_link", rospy.Time(0), rospy.Duration(0.1))

            msg = PoseWithCovarianceStamped()
            # Use the transform's timestamp so EKF sees correct timing
            msg.header.stamp = trans.header.stamp if hasattr(trans, 'header') else rospy.Time.now()
            msg.header.frame_id = "map"   # groundtruth expressed in map frame

            # Fill in pose
            msg.pose.pose.position.x = trans.transform.translation.x
            msg.pose.pose.position.y = trans.transform.translation.y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation = trans.transform.rotation

            # More realistic covariance (diag, row-major 6x6)
            # [x, y, z, rot_x, rot_y, rot_z]
            cov = [0.0] * 36
            cov[0]  = 0.01    # var(x)  = 0.01 m^2  -> std = 0.1 m
            cov[7]  = 0.01    # var(y)
            cov[14] = 0.01    # var(z) (not used in 2D)
            cov[21] = 1e-6    # var(roll)  (very small)
            cov[28] = 1e-6    # var(pitch) (very small)
            cov[35] = 0.05    # var(yaw) = 0.05 rad^2 -> std ~0.22 rad (~12.8 deg)
            msg.pose.covariance = cov

            self.pub.publish(msg)

        except Exception as e:
            rospy.logwarn_throttle(5.0, "Groundtruth TF lookup failed: %s", str(e))


if __name__ == "__main__":
    gp = GroundtruthPublisher()
    rospy.spin()
