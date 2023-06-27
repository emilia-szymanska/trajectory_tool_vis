import numpy as np
from scipy.spatial import Delaunay
import shapely.geometry as geometry
from shapely.ops import cascaded_union, polygonize
import math
import csv
import tf
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from math import tan


def alpha_shape(points, alpha):

    if len(points) < 4:
        return geometry.MultiPoint(list(points)).convex_hull

    def add_edge(edges, edge_points, coords, i, j):
        """
        Add a line between the i-th and j-th points,
        if not in the list already
        """
        if (i, j) in edges or (j, i) in edges:
            # already added
            return
        edges.add( (i, j) )
        edge_points.append(coords[ [i, j] ])

    coords = np.array([point.coords[0] for point in points])

    tri = Delaunay(coords)
    edges = set()
    edge_points = []
    # loop over triangles:
    # ia, ib, ic = indices of corner points of the triangle
    for ia, ib, ic in tri.vertices:
        pa = coords[ia]
        pb = coords[ib]
        pc = coords[ic]
        # Lengths of sides of triangle
        a = math.sqrt((pa[0]-pb[0])**2 + (pa[1]-pb[1])**2)
        b = math.sqrt((pb[0]-pc[0])**2 + (pb[1]-pc[1])**2)
        c = math.sqrt((pc[0]-pa[0])**2 + (pc[1]-pa[1])**2)
        # Semiperimeter of triangle
        s = (a + b + c)/2.0
        # Area of triangle by Heron's formula
        area = math.sqrt(s*(s-a)*(s-b)*(s-c))
        circum_r = a*b*c/(4.0*area)
        # Here's the radius filter.
        if circum_r < 1.0/alpha:
            add_edge(edges, edge_points, coords, ia, ib)
            add_edge(edges, edge_points, coords, ib, ic)
            add_edge(edges, edge_points, coords, ic, ia)

    m = geometry.MultiLineString(edge_points)
    triangles = list(polygonize(m))
    return cascaded_union(triangles), edge_points

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

p0 = (5, 40, 0.0)
q0 = (0.0, 0.0, -0.123, 1)
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

pts = [geometry.Point(pose.position.x, pose.position.y) for pose in points.poses]
concave_hull, edge_points = alpha_shape(pts, alpha=1)


fig, ax = plt.subplots()

for geom in concave_hull.geoms: 
    # extract x and y coordinates from the polygon (concave hull)
    polygon_path = geom.exterior.xy

    # plot the polygon
    ax.fill(polygon_path[0], polygon_path[1], alpha=0.5, fc='c', ec='none')


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

plt.title('Concave hull')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()