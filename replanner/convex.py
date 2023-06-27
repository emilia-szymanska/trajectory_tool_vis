import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull
import shapely.geometry as geometry
from shapely.geometry import Polygon
from shapely.ops import cascaded_union, polygonize
import math
import csv
import tf
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from math import tan
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


def convex_hull(points):
    points_array = np.array([[pose.position.x, pose.position.y] for pose in points])
    hull = ConvexHull(points_array)
    return Polygon(points_array[hull.vertices])

def read_csv(filename):
    csvfile = open(filename)
    csvreader = csv.reader(csvfile)
    header = next(csvreader)
    rows = []
    for row in csvreader:
        tmp = {}
        for key, val in zip(header, row):
            tmp[key] = val
        rows.append(tmp)
    current = []
    for row in rows:
        p = (row['x'], row['y'], row['z'])
        q = (row['qx'], row['qy'], row['qz'], row['qw'])
        pose = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(p),
            tf.transformations.quaternion_matrix(q))
        current.append(pose)
    return current

def robot_corners(robot_width, robot_length, robot_position, robot_orientation):
    robot_corners_val = np.array([
        [-robot_length/2, -robot_width/2],
        [robot_length/2, -robot_width/2],
        [robot_length/2, robot_width/2],
        [-robot_length/2, robot_width/2]
    ])
    rotation_matrix = np.array([
        [np.cos(robot_orientation), -np.sin(robot_orientation)],
        [np.sin(robot_orientation), np.cos(robot_orientation)]
    ])
    robot_cor = np.dot(robot_corners_val, rotation_matrix.T) + robot_position
    return robot_cor

#############################################################################33

p0 = (5, 40, 0.0)
q0 = (0.0, 0.0, -0.123, 1)
converted = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(p0),
        tf.transformations.quaternion_matrix(q0)
    )
poses = read_csv("smoothed.csv")

global_poses = PoseArray()
for i, transform in enumerate(poses):
    converted = converted @ transform
    topush = Pose()
    topush.orientation = Quaternion(*tf.transformations.quaternion_from_matrix(converted))
    topush.position = Point(*tf.transformations.translation_from_matrix(converted))
    global_poses.poses.append(topush)

fov_hor = 2.27
fov_ver = 0.52
height = 3.0
robot_width = 2.0*height*tan(fov_hor/2)
robot_length = 2.0*height*tan(fov_ver/2)

points = PoseArray()
for i, pose in enumerate(global_poses.poses):
    if i > 400:
        break

    position = np.array([pose.position.x, pose.position.y])
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    robot = robot_corners(robot_width, robot_length, position, yaw)
    
    for pair in robot:
        tmp = Pose()
        tmp.position.x = pair[0]
        tmp.position.y = pair[1]
        points.poses.append(tmp)

hull = convex_hull(points.poses)

fig, ax = plt.subplots()

for i, pose in enumerate(global_poses.poses):
    if i < 400:
        position = np.array([pose.position.x, pose.position.y])

        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        orientation = np.array([np.cos(euler[2]), np.sin(euler[2])])

        # Normalize orientation to length 1
        orientation /= np.linalg.norm(orientation)

        ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1, color='k')
    else:
        # extract x and y coordinates from the points
        # points_x = [point.position.x for point in global_poses.poses]
        # points_y = [point.position.y for point in global_poses.poses]

        # plot the points
        
        ax.plot(pose.position.x, pose.position.y, 'ro', markersize=1) # 'ko' makes the points black (k) and circular (o)

polygon_path = hull.exterior.xy
ax.fill(polygon_path[0], polygon_path[1], alpha=0.5, fc='c', ec='none')

plt.title('Convex hull')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()