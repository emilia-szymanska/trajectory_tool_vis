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

def convex_hull(points):
    """
    Compute the convex hull of a set of points.

    :param points: List of geometry_msgs.msg.Pose objects.
    :return: shapely.geometry.Polygon object representing the convex hull.
    """
    # Convert Pose objects to numpy array
    points_array = np.array([[pose.position.x, pose.position.y] for pose in points])
    
    # Compute convex hull
    hull = ConvexHull(points_array)
    
    # Convert convex hull to Polygon and return
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

p0 = (5, 40, 0.0)
q0 = (0.0, 0.0, -0.125, 1)
converted = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(p0),
        tf.transformations.quaternion_matrix(q0)
    )
poses = read_csv("smoothed.csv")

global_poses = PoseArray()
for transform in poses:
    converted = converted @ transform
    topush = Pose()
    topush.orientation = Quaternion(*tf.transformations.quaternion_from_matrix(converted))
    topush.position = Point(*tf.transformations.translation_from_matrix(converted))
    global_poses.poses.append(topush)

# points = [geometry.Point(pose.position.x, pose.position.y) for pose in global_poses.poses]

hull = convex_hull(global_poses.poses)

print(hull)

import matplotlib.pyplot as plt

plt.figure()
polygon_path = hull.exterior.xy
plt.fill(polygon_path[0], polygon_path[1], alpha=0.5, fc='c', ec='none')


# extract x and y coordinates from the points
points_x = [point.position.x for point in global_poses.poses]
points_y = [point.position.y for point in global_poses.poses]

# plot the points
plt.plot(points_x, points_y, 'ko') # 'ko' makes the points black (k) and circular (o)

plt.title('Concave hull and Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()