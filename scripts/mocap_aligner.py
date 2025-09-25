#!/usr/bin/env python3
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped

class MocapAligner:
    def __init__(self):
        rospy.init_node("mocap_aligner")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.offset_set = False
        self.offset_trans = None
        self.offset_rot = None

        rospy.Timer(rospy.Duration(0.01), self.broadcast_aligned)

    def broadcast_aligned(self, event):
        try:
            # Get mocap_laser_link and base_link relative to mocap frame
            t_mocap = self.tf_buffer.lookup_transform("mocap", "mocap_laser_link", rospy.Time(0))
            t_base  = self.tf_buffer.lookup_transform("mocap", "base_link", rospy.Time(0))

            if not self.offset_set:
                # Compute offset at t=0: base_link - mocap_laser_link
                dx = t_base.transform.translation.x - t_mocap.transform.translation.x
                dy = t_base.transform.translation.y - t_mocap.transform.translation.y
                dz = t_base.transform.translation.z - t_mocap.transform.translation.z

                # Simple: just shift translations (ignoring rotation for now)
                self.offset_trans = (dx, dy, dz)
                self.offset_rot   = tft.quaternion_multiply(
                    [t_base.transform.rotation.x,
                     t_base.transform.rotation.y,
                     t_base.transform.rotation.z,
                     t_base.transform.rotation.w],
                    tft.quaternion_inverse([
                        t_mocap.transform.rotation.x,
                        t_mocap.transform.rotation.y,
                        t_mocap.transform.rotation.z,
                        t_mocap.transform.rotation.w
                    ])
                )
                self.offset_set = True
                rospy.loginfo("Offset initialized!")

            if self.offset_set:
                # Apply offset to mocap_laser_link
                trans = TransformStamped()
                trans.header.stamp = rospy.Time.now()
                trans.header.frame_id = "mocap"
                trans.child_frame_id = "mocap_aligner"

                trans.transform.translation.x = t_mocap.transform.translation.x + self.offset_trans[0]
                trans.transform.translation.y = t_mocap.transform.translation.y + self.offset_trans[1]
                trans.transform.translation.z = t_mocap.transform.translation.z + self.offset_trans[2]

                q_mocap = [
                    t_mocap.transform.rotation.x,
                    t_mocap.transform.rotation.y,
                    t_mocap.transform.rotation.z,
                    t_mocap.transform.rotation.w
                ]

                q_aligned = tft.quaternion_multiply(self.offset_rot, q_mocap)
                trans.transform.rotation.x = q_aligned[0]
                trans.transform.rotation.y = q_aligned[1]
                trans.transform.rotation.z = q_aligned[2]
                trans.transform.rotation.w = q_aligned[3]

                self.tf_broadcaster.sendTransform(trans)

        except Exception as e:
            rospy.logwarn_throttle(5, "TF lookup failed: %s", str(e))

if __name__ == "__main__":
    MocapAligner()
    rospy.spin()
