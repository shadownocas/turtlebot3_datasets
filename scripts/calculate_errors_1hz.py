#!/usr/bin/env python3
import rospy
import csv
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import bisect

# -----------------------
# Parameters
csv_file = '/home/ines/robotica/intro_robotics/src/turtlebot3_datasets/data/ekf_error.csv'

# Create CSV with header
with open(csv_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'error_m', 'std_x', 'std_y'])

# -----------------------
latest_ekf = None
gt_times = []
gt_poses = []

def gt_callback(msg: Path):
    global gt_times, gt_poses
    # Store ground-truth poses and timestamps
    gt_times = [pose.header.stamp.to_sec() for pose in msg.poses]
    gt_poses = msg.poses

def ekf_callback(msg: Odometry):
    global gt_times, gt_poses

    if not gt_times:
        return  # wait until ground-truth path is received

    t = msg.header.stamp.to_sec()
    # Find closest ground-truth index using bisect
    idx = bisect.bisect_left(gt_times, t)
    if idx >= len(gt_times):
        idx = -1
    gt_pose: PoseStamped = gt_poses[idx]

    # Compute position error
    x_err = msg.pose.pose.position.x - gt_pose.pose.position.x
    y_err = msg.pose.pose.position.y - gt_pose.pose.position.y
    pos_err = np.sqrt(x_err**2 + y_err**2)

    # Uncertainty from EKF covariance
    cov = msg.pose.covariance
    std_x = np.sqrt(cov[0])
    std_y = np.sqrt(cov[7])

    # Write to CSV
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([t, pos_err, std_x, std_y])

    #rospy.loginfo(f"Time {t:.2f} | Error: {pos_err:.3f} m | Std x: {std_x:.3f} y: {std_y:.3f}")

# -----------------------
rospy.init_node('ekf_error_logger')
rospy.Subscriber('/groundtruth_path', Path, gt_callback)
rospy.Subscriber('/odometry/filtered', Odometry, ekf_callback)

rospy.spin()
