import numpy as np
import csv
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
import matplotlib.pyplot as plt


csvfile = open("../data/smoothed.csv")
csvreader = csv.reader(csvfile)
header = next(csvreader)

rows = []
for row in csvreader:
    tmp = {}
    for key, val in zip(header, row):
        tmp[key] = val
    rows.append(tmp)

# p0 = (-7.96349, 29.8067, 0.0)
# q0 = (0.0, 0.0, -0.044864, 0.998993)

####
p0 = (5, 40, 0.0)
q0 = (0.0, 0.0, -0.123, 1)
####


converted = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(p0),
        tf.transformations.quaternion_matrix(q0)
    )

current = []
for row in rows:
    p = (row['x'], row['y'], row['z'])
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

fig, ax = plt.subplots()

for pose in global_poses.poses:
    position = np.array([pose.position.x, pose.position.y])

    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    orientation = np.array([np.cos(euler[2]), np.sin(euler[2])])

    # Normalize orientation to length 1
    orientation /= np.linalg.norm(orientation)

    ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1)


ax.set_xlabel('X')
ax.set_ylabel('Y')

plt.show()

