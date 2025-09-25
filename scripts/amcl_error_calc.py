#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np
from collections import deque
import csv
import os

class PathErrorCalculator:
    def __init__(self):
        rospy.init_node("path_error_calculator")

        # Parameters
        self.window_size = rospy.get_param("~window_size", 10)
        self.csv_file = rospy.get_param("~csv_file", "/home/ines/robotica/intro_robotics/src/turtlebot3_datasets/data/amcl_error.csv")
        self.log_interval = rospy.get_param("~log_interval", 1.0)  # seconds (set 0.5 for half-second)

        # Subscribers
        rospy.Subscriber("/groundtruth_path", Path, self.gt_callback)
        rospy.Subscriber("/amcl_path", Path, self.amcl_callback)

        # Publishers
        self.inst_error_pub = rospy.Publisher("/path_error_instant", Float32, queue_size=10)
        self.cum_rmse_pub = rospy.Publisher("/path_error_cumulative", Float32, queue_size=10)
        self.window_rmse_pub = rospy.Publisher("/path_error_window", Float32, queue_size=10)

        # Store latest paths
        self.gt_path = None
        self.amcl_path = None

        # Sliding window
        self.error_window = deque(maxlen=self.window_size)

        # CSV setup
        self.start_time = rospy.get_time()
        self.csv_path = os.path.join(os.getcwd(), self.csv_file)
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_f)
        self.csv_writer.writerow(["time_sec", "instant_error"])

        # Downsampling control
        self.last_log_time = self.start_time

        rospy.loginfo(f"Logging errors every {self.log_interval}s to {self.csv_path}")

    def gt_callback(self, msg):
        self.gt_path = msg
        self.calculate_errors()

    def amcl_callback(self, msg):
        self.amcl_path = msg
        self.calculate_errors()

    def calculate_errors(self):
        if self.gt_path is None or self.amcl_path is None:
            return
        if len(self.gt_path.poses) == 0 or len(self.amcl_path.poses) == 0:
            return

        # --- Cumulative RMSE ---
        n = min(len(self.gt_path.poses), len(self.amcl_path.poses))
        cum_errors = []
        for i in range(n):
            dx = self.gt_path.poses[i].pose.position.x - self.amcl_path.poses[i].pose.position.x
            dy = self.gt_path.poses[i].pose.position.y - self.amcl_path.poses[i].pose.position.y
            cum_errors.append(np.sqrt(dx**2 + dy**2))
        cum_rmse = np.sqrt(np.mean(np.array(cum_errors)**2))
        self.cum_rmse_pub.publish(cum_rmse)

        # --- Instantaneous error ---
        amcl_pose = self.amcl_path.poses[-1]
        closest_gt = min(
            self.gt_path.poses,
            key=lambda p: abs((p.header.stamp - amcl_pose.header.stamp).to_sec())
        )
        dx = closest_gt.pose.position.x - amcl_pose.pose.position.x
        dy = closest_gt.pose.position.y - amcl_pose.pose.position.y
        inst_error = np.sqrt(dx**2 + dy**2)
        self.inst_error_pub.publish(inst_error)

        # --- Sliding window RMSE ---
        self.error_window.append(inst_error)
        window_rmse = np.sqrt(np.mean(np.array(self.error_window)**2))
        self.window_rmse_pub.publish(window_rmse)

        # --- Save to CSV only if enough time passed ---
        now = rospy.get_time()
        if now - self.last_log_time >= self.log_interval:
            rel_time = now - self.start_time
            self.csv_writer.writerow([round(rel_time, 2), inst_error])
            self.csv_f.flush()
            self.last_log_time = now

            rospy.loginfo(
                f"t={rel_time:.1f}s | Instant: {inst_error:.3f} m | "
                f"Window({len(self.error_window)}): {window_rmse:.3f} m | "
                f"Cumulative: {cum_rmse:.3f} m"
            )

if __name__ == "__main__":
    PathErrorCalculator()
    rospy.spin()
