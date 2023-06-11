import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import box, Polygon
import numpy as np
import csv
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
import matplotlib.pyplot as plt
from math import tan

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

def update_grid(grid, robot_width, robot_length, robot_position, robot_orientation):
    robot_corners = np.array([
        [-robot_length/2, -robot_width/2],
        [robot_length/2, -robot_width/2],
        [robot_length/2, robot_width/2],
        [-robot_length/2, robot_width/2]
    ])
    rotation_matrix = np.array([
        [np.cos(robot_orientation), -np.sin(robot_orientation)],
        [np.sin(robot_orientation), np.cos(robot_orientation)]
    ])
    robot_corners = np.dot(robot_corners, rotation_matrix.T) + robot_position
    robot = Polygon(robot_corners)

    min_indices = np.floor(np.min(robot_corners, axis=0) / grid_cell_size).astype(int)
    max_indices = np.ceil(np.max(robot_corners, axis=0) / grid_cell_size).astype(int)
    
    # Check each cell if it is intersecting with the robot
    for i in range(max(0, min_indices[0]), min(num_cells[0], max_indices[0])):
        for j in range(max(0, min_indices[1]), min(num_cells[1], max_indices[1])):
            cell_corners = np.array([
                [i*grid_cell_size, j*grid_cell_size],
                [(i+1)*grid_cell_size, j*grid_cell_size],
                [(i+1)*grid_cell_size, (j+1)*grid_cell_size],
                [i*grid_cell_size, (j+1)*grid_cell_size]
            ])
            cell = Polygon(cell_corners)
            if robot.intersects(cell):
                grid[j, i] = 1

def plot_grid(grid):
    plt.figure(figsize=(6,6)) # size of the figure
    plt.imshow(grid, cmap='hot', interpolation='nearest')
    plt.colorbar(label='Value') # Display a colorbar
    plt.xlabel('X') # Label for the x-axis
    plt.ylabel('Y') # Label for the y-axis
    plt.title('Heatmap') # Title of the heatmap
    plt.show()

grid_cell_size = 0.1
grid_size = np.array([50.0, 50.0])  # size of the grid (width, height)
num_cells = (grid_size / grid_cell_size).astype(int)
grid = np.zeros(num_cells, dtype=int)

fov_hor = 2.27
fov_ver = 0.52
height = 3.0
robot_width = 2.0*height*tan(fov_hor/2)
robot_length = 2.0*height*tan(fov_ver/2)

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

for i, pose in enumerate(global_poses.poses):
    if i >= 200:
        break
    position = np.array([pose.position.x, pose.position.y])
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    update_grid(grid, robot_width, robot_length, position, yaw)

plot_grid(grid)