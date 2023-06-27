import numpy as np
import csv
import matplotlib.pyplot as plt


csvfile = open("gtsam_pose.csv")
csvreader = csv.reader(csvfile, delimiter='\t')
header = next(csvreader)


rows = []
for row in csvreader:
    tmp = {}
    for key, val in zip(header, row):
        tmp[key] = val
    rows.append(tmp)


fig, ax = plt.subplots()
for i, row in enumerate(rows):
    if i % 2 == 0 or i % 3 != 0:
        continue

    yaw = -float(row['ori_z'])
    position = np.array([float(row['pos_x']), float(row['pos_y'])])
    orientation = np.array([np.cos(yaw), np.sin(yaw)])
    
    # Normalize orientation to length 1
    orientation /= np.linalg.norm(orientation)

    ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1)


ax.set_xlabel('X')
ax.set_ylabel('Y')

plt.show()

