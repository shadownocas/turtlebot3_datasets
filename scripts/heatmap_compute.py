import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# Path to folder containing CSVs
csv_dir = "/home/ines/robotica/intro_robotics/src/turtlebot3_datasets/data"

# List of files
files = [
    "ekf_error_Q0.1_R0.1.csv",
    "ekf_error_Q0.5_R0.01.csv",
    "ekf_error_Q1.0_R0.05.csv"
]

# Corresponding Q and R values
Q_values = [0.1, 0.5, 1.0]
R_values = [0.1, 0.01, 0.05]

# Create RMSE matrix
rmse_matrix = np.zeros((len(Q_values), len(R_values)))

for i, Q in enumerate(Q_values):
    for j, R in enumerate(R_values):
        filename = f"ekf_error_Q{Q}_R{R}.csv"
        filepath = os.path.join(csv_dir, filename)
        
        if os.path.exists(filepath):
            df = pd.read_csv(filepath)
            rmse_matrix[i, j] = df['error_m'].mean()  # mean position error
        else:
            rmse_matrix[i, j] = np.nan  # missing file


plt.figure(figsize=(6,5))
plt.imshow(rmse_matrix, origin='lower', cmap='viridis',
           extent=[min(R_values), max(R_values), min(Q_values), max(Q_values)],
           aspect='auto')
plt.colorbar(label='Mean position error [m]')
plt.xlabel('Measurement noise R')
plt.ylabel('Process noise Q')
plt.title('EKF performance heatmap')
plt.savefig(os.path.join(csv_dir, 'ekf_heatmap.png'))
plt.show()
