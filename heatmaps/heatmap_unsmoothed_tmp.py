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
from tqdm import tqdm

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
                grid[j, i] += 1

def update_path(grid, position):
    x = np.floor(np.min(position[0], axis=0) / grid_cell_size).astype(int)
    y = np.floor(np.min(position[1], axis=0) / grid_cell_size).astype(int)
    if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
        grid[y-4:y+4, x-4:x+4] = 50

def plot_grid(grid):
    plt.figure(figsize=(6,6)) # size of the figure
    plt.imshow(grid, cmap='hot', interpolation='nearest', vmin=0, vmax=75, origin='lower')
    plt.colorbar(label='Value') # Display a colorbar
    plt.xlabel('X') # Label for the x-axis
    plt.ylabel('Y') # Label for the y-axis
    plt.title('Heatmap') # Title of the heatmap
    plt.show()

grid_cell_size = 0.1
grid_size = np.array([50.0, 50.0])  # size of the grid (width, height)
num_cells = (grid_size / grid_cell_size).astype(int)
grid = np.zeros(num_cells, dtype=int)

fov_hor = 1.0
fov_ver = 0.52
height = 3.0
robot_width = 2.0*height*tan(fov_hor/2)
robot_length = 2.0*height*tan(fov_ver/2)

p0 = (5, 40, 0.0)
q0 = (0.0, 0.0, -0.07, 1)
converted = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(p0),
        tf.transformations.quaternion_matrix(q0)
    )

poses = read_csv("../data/unsmoothed_tmp.csv")

n = 10
global_poses = PoseArray()
for i, transform in enumerate(poses):
    # if i > 100:
    #     break
    converted = converted @ transform
    topush = Pose()
    topush.orientation = Quaternion(*tf.transformations.quaternion_from_matrix(converted))
    topush.position = Point(*tf.transformations.translation_from_matrix(converted))
    
    if i > 0 and topush.orientation != global_poses.poses[-1].orientation:
        pose1 = global_poses.poses[-1]
        q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        q2 = [topush.orientation.x, topush.orientation.y, topush.orientation.z, topush.orientation.w]

        # interpolate positions and orientations separately
        for i in range(n+1):
            t = float(i) / float(n+1)  # interpolation factor

            # spherical linear interpolation for orientation
            q = tf.transformations.quaternion_slerp(q1, q2, t)

            # create a Pose message
            pose_tmp = Pose()
            pose_tmp.position.x = topush.position.x
            pose_tmp.position.y = topush.position.y
            pose_tmp.position.z = topush.position.z
            pose_tmp.orientation.x = q[0]
            pose_tmp.orientation.y = q[1]
            pose_tmp.orientation.z = q[2]
            pose_tmp.orientation.w = q[3]

            global_poses.poses.append(pose_tmp)
    global_poses.poses.append(topush)

# print(global_poses.poses)
print(grid)
for pose in tqdm(global_poses.poses):
    position = np.array([pose.position.x, pose.position.y])
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    # update_path(grid, position)
    update_grid(grid, robot_width, robot_length, position, yaw)

print("Path")
for pose in tqdm(global_poses.poses):
    position = np.array([pose.position.x, pose.position.y])
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    update_path(grid, position)

cropped = grid[85:390, 115:420]

print(f"Zero: {np.count_nonzero(cropped==0)}")
print(f"Non zero: {np.count_nonzero(cropped)}")
print(f"Total: {cropped.size}")
print(f"Coverage {100 * round(np.count_nonzero(cropped) / cropped.size, 6)}%")

plot_grid(grid)