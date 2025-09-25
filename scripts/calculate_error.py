#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
import csv
import numpy as np
import os

# Parameters
csv_file = rospy.get_param('~csv_file', '/home/ines/robotica/intro_robotics/src/turtlebot3_datasets/data/ekf3_error.csv') 
groundtruth_topic = rospy.get_param('~groundtruth_topic', '/groundtruth_path')
estimated_topic   = rospy.get_param('~estimated_topic', '/estimated_path')

gt_poses = []
est_poses = []

# Ensure CSV exists and write header
if not os.path.exists(csv_file):
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'error_mm'])

def groundtruth_cb(msg):
    global gt_poses
    gt_poses = msg.poses

def estimated_cb(msg):
    global est_poses
    est_poses = msg.poses

def compute_errors(event=None):
    """Compute error for all available poses and append to CSV."""
    N = min(len(gt_poses), len(est_poses))
    if N == 0:
        return  # No poses yet

    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        # Compute only for new poses (avoid duplicates)
        start_index = getattr(compute_errors, 'last_index', 0)
        for i in range(start_index, N):
            gt = gt_poses[i].pose.position
            est = est_poses[i].pose.position
            dx = est.x - gt.x
            dy = est.y - gt.y
            error = np.sqrt(dx**2 + dy**2)
            time_sec = gt_poses[i].header.stamp.to_sec()
            writer.writerow([time_sec, error * 1000])  # convert to mm
            rospy.loginfo('t={:.2f}, error={:.2f} mm'.format(time_sec, error*1000))
        compute_errors.last_index = N  # remember last processed index

if __name__ == "__main__":
    rospy.init_node('calculate_error')

    rospy.Subscriber(groundtruth_topic, Path, groundtruth_cb)
    rospy.Subscriber(estimated_topic, Path, estimated_cb)

    # Timer to compute errors every 0.5s
    rospy.Timer(rospy.Duration(0.5), compute_errors)

    rospy.loginfo("Calculate_error node started. Computing errors continuously...")
    rospy.spin()
