import numpy as np
import csv
import matplotlib.pyplot as plt
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point

csvfile = open("../data/unsmoothed_gtsam_pose.csv")
csvreader = csv.reader(csvfile, delimiter='\t')
header = next(csvreader)


rows = []
for row in csvreader:
    tmp = {}
    for key, val in zip(header, row):
        tmp[key] = float(val)
    rows.append(tmp)


fig, ax = plt.subplots()
for i, row in enumerate(rows):
    if i > 3050:
        break
    if i % 2 == 0 or i % 3 != 0:
        continue

    yaw = -row['ori_z']
    position = np.array([row['pos_x'], row['pos_y']])
    orientation = np.array([np.cos(yaw), np.sin(yaw)])
    
    # Normalize orientation to length 1
    orientation /= np.linalg.norm(orientation)

    ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1)


p0 = (rows[0]['pos_x'], rows[0]['pos_y'], 0.0)
q0 = (rows[0]['qx'], rows[0]['qy'], rows[0]['qz'], rows[0]['qw'])

csvfile = open("../data/unsmoothed.csv")
csvreader = csv.reader(csvfile)
header = next(csvreader)

rows = []
for i, row in enumerate(csvreader):
    if i > 17: 
        tmp = {}
        for key, val in zip(header, row):
            tmp[key] = val
        rows.append(tmp)


converted = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(p0),
        tf.transformations.quaternion_matrix(q0)
    )
print(p0)
print(q0)

current = []
for row in rows:
    p = (row['x'], row['y'], 0.0)
    q = (row['qx'], row['qy'], row['qz'], row['qw'])
    pose = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(p),
        tf.transformations.quaternion_matrix(q))
    current.append(pose)

global_poses = PoseArray()
for transform in current:
    converted = converted @ transform

    topush = Pose()
    topush.orientation = Quaternion(*tf.transformations.quaternion_from_matrix(converted))
    topush.position = Point(*tf.transformations.translation_from_matrix(converted))
    global_poses.poses.append(topush)

point_x = []
point_y = []
for pose in global_poses.poses:
    # position = np.array([pose.position.x, pose.position.y])

    # quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    # euler = euler_from_quaternion(quaternion)
    # orientation = np.array([np.cos(euler[2]), np.sin(euler[2])])

    # # Normalize orientation to length 1
    # orientation /= np.linalg.norm(orientation)

    # ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1, color="r")
    point_x.append(pose.position.x)
    point_y.append(pose.position.y)

ax.plot(point_x, point_y, 'r')

ax.set_xlabel('X')
ax.set_ylabel('Y')

plt.show()

