#!/usr/bin/env python3
import rospy
import csv
import os
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

# -----------------------
# Parameters
csv_dir = '/home/ines/robotica/intro_robotics/src/turtlebot3_datasets/data'
csv_file = os.path.join(csv_dir, 'groundtruth_1hz.csv')
frame_id = 'odom'

# Ensure data directory exists
os.makedirs(csv_dir, exist_ok=True)

# Create CSV file with header if missing
if not os.path.isfile(csv_file):
    rospy.loginfo(f"CSV file not found. Creating: {csv_file}")
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

# Open CSV for appending
csv_fh = open(csv_file, 'a', newline='')
writer = csv.writer(csv_fh)

# -----------------------
# ROS node and publishers
rospy.init_node('groundtruth_path_publisher')
path_pub = rospy.Publisher('/groundtruth_path', Path, queue_size=10)
path = Path()
path.header.frame_id = frame_id

# -----------------------
# Global variable to store latest odometry
latest_odom = None

def odom_callback(msg):
    global latest_odom
    latest_odom = msg

rospy.Subscriber("/odom", Odometry, odom_callback)

# -----------------------
# Main loop at 1 Hz
rate = rospy.Rate(1)  # 1 Hz
rospy.loginfo("Publishing ground-truth path at 1 Hz...")

while not rospy.is_shutdown():
    if latest_odom is not None:
        odom = latest_odom
        pose = odom.pose.pose
        stamp = odom.header.stamp

        # Write to CSV
        writer.writerow([
            stamp.to_sec(),
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        csv_fh.flush()

        # Append to Path and publish
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = stamp
        pose_stamped.pose = pose
        path.poses.append(pose_stamped)
        path_pub.publish(path)

    rate.sleep()
