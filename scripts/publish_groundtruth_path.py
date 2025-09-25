#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TFPathRecorder:
    def __init__(self):
        rospy.init_node("publish_groundtruth_path", anonymous=True)

        # tf groundtruth 

        self.path = Path() # Actually store the path here
        self.path.header.frame_id = "map"
        self.pub = rospy.Publisher('/groundtruth_path', Path, queue_size=10)
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Timer to record path (downsampled to 1Hz)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.record_tf_path)  # Changed from 0.1 to 1.0

    def record_tf_path(self, event):
        try:
            trans = self.tf_buffer.lookup_transform("map" , "mocap_laser_link", rospy.Time())
            
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = trans.transform.rotation
            
            self.path.header.stamp = rospy.Time.now()
            self.path.poses.append(pose)
            self.pub.publish(self.path)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "TF lookup failed: %s", str(e))

if __name__ == "__main__":
    TFPathRecorder()
    rospy.spin()