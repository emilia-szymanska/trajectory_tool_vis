import numpy as np
import csv
import matplotlib.pyplot as plt
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
import numpy as np

import numpy as np
from sklearn.neighbors import NearestNeighbors
import random

smoothed = 1

if smoothed == 1:
    RESULTS_FILE = "../data/smoothed_gtsam_pose.csv"
    GT_FILE = "../data/smoothed_multiple.csv"
    x1 = 3
    y1 = 29.0
    it0 = 9
    output_file = "../data/errors_smoothed.csv"
    output_file_x = "../data/errors_x_smoothed.csv"
    output_file_y = "../data/errors_y_smoothed.csv"
else:
    RESULTS_FILE = "../data/unsmoothed_gtsam_pose.csv"
    GT_FILE = "../data/unsmoothed_multiple.csv"
    x1 = 0.8
    y1 = 29.0
    it0 = 0
    output_file = "../data/errors_unsmoothed.csv"
    output_file_x = "../data/errors_x_unsmoothed.csv"
    output_file_y = "../data/errors_y_unsmoothed.csv"

desired_length = 2700

holes = [[[6.0,  20.0],
          [5.0,  23.5],
          [8.0,  24.0],
          [12.0, 23.0],
          [9.0,  21.0]],
         [[22.0,  7.0],
          [22.0, 13.0],
          [26.0, 14.0],
          [24.0, 11.0],
          [24.0,  7.0]]]



def write_to_file(data, filename):
    with open(filename, 'w') as file:
        for item in data:
            file.write(f"{item}\n")

def random_discard(input_list, target_length):
    """
    Randomly discard elements from a list to reach a specified length.

    Parameters:
    input_list (list): The list to reduce.
    target_length (int): The target length of the list.

    Returns:
    list: The input list reduced to the target length, with elements randomly removed.
    """
    # Check if the target length is greater than or equal to the length of the list
    if target_length >= len(input_list):
        print("Target length is greater than or equal to list length. Returning original list.")
        return input_list

    # Randomly select 'target_length' elements from the list without replacement
    reduced_list = random.sample(input_list, target_length)

    return reduced_list

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform between corresponding 2D points A and B.
    '''
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = np.dot(AA.T, BB)

    U, S, Vt = np.linalg.svd(H)

    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
       Vt[3,:] *= -1
       R = np.dot(Vt.T, U.T)

    t = centroid_B.T - np.dot(R, centroid_A.T)

    return R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    '''
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()

def icp(A, B, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: aligns B to A
    '''
    src = np.array(B)
    dst = np.array(A)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src, dst)

        # compute the transformation between the current source and nearest destination points
        R, t = best_fit_transform(src, dst[indices])

        # update the current source
        t_reshaped = t.reshape(-1,1)
        src = (np.dot(R, src.T) + t_reshaped).T

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    R, t = best_fit_transform(B, src)

    return R, t

def yaw_to_quaternion(yaw):
    """ Convert yaw angle to quaternion. """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    q = [cy, 0, 0, sy]
    return q

def compute_errors(experiment, transformed_ground_truth):
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(transformed_ground_truth)

    distances, _ = neigh.kneighbors(experiment)
    
    return distances.ravel()

def normalize_trajectory(trajectory, first_point=(0,0), it = 0):
    # Initialize an empty list to hold the transformed poses
    normalized_trajectory = []

    # Get the first pose
    x0, y0, theta0 = trajectory[it]

    # Calculate the rotation to align the first pose's direction with the x axis
    rotation = -theta0

    x_first, y_first = first_point

    # Transform each pose in the trajectory
    for x, y, theta in trajectory:
        # Translate the pose so that the first pose is at the origin
        x -= x0
        y -= y0

        # Rotate the pose so that the first pose's direction is along the x axis
        x_new = x * np.cos(rotation) - y * np.sin(rotation)
        y_new = x * np.sin(rotation) + y * np.cos(rotation)
        theta_new = theta + rotation

        x_new += x_first
        y_new += y_first

        # Append the transformed pose to the new trajectory
        normalized_trajectory.append((x_new, y_new, theta_new))

    return normalized_trajectory

def compute_xy_errors(experiment, transformed_ground_truth):
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(transformed_ground_truth)

    _, indices = neigh.kneighbors(experiment)
    
    closest_points = np.array(transformed_ground_truth)[indices.flatten()]

    experiment_np = np.array(experiment)
    
    x_differences = experiment_np[:, 0] - closest_points[:, 0]
    y_differences = experiment_np[:, 1] - closest_points[:, 1]

    return x_differences, y_differences

csvfile = open(RESULTS_FILE)
csvreader = csv.reader(csvfile, delimiter='\t')
header = next(csvreader)

rows = []
for row in csvreader:
    tmp = {}
    for key, val in zip(header, row):
        tmp[key] = float(val)
    rows.append(tmp)

fig, ax = plt.subplots()

trajectory_result = []
for i, row in enumerate(rows):
    
    yaw = -row['ori_z']
    x = row['pos_x']
    y = row['pos_y']
    trajectory_result.append([x,y,yaw])


normalized = normalize_trajectory(trajectory_result, (x1,y1), it=it0)

experiment = []
for i, pose in enumerate(normalized):
    
    yaw = pose[2]
    position = np.array([pose[0], pose[1]])
    orientation = np.array([np.cos(yaw), np.sin(yaw)])
    
    # Normalize orientation to length 1
    orientation /= np.linalg.norm(orientation)

    if i % 2 != 0 and i % 3 == 0:
        ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1)
    experiment.append([pose[0], pose[1]])

q = yaw_to_quaternion(normalized[0][2])
p0 = (normalized[0][0], normalized[0][1], 0.0)
# q0 = (q[0], q[1], q[2], q[3])
q0 = (0,0,0,1)

#############################################
csvfile = open(GT_FILE)
csvreader = csv.reader(csvfile)
header = next(csvreader)

rows = []
for i, row in enumerate(csvreader):
    if i > 84: 
        tmp = {}
        for key, val in zip(header, row):
            tmp[key] = val
        rows.append(tmp)

converted = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(p0),
        tf.transformations.quaternion_matrix(q0)
    )

ground_truth = []
for row in rows:
    p = (row['x'], row['y'], 0.0)
    q = (row['qx'], row['qy'], row['qz'], row['qw'])
    pose = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(p),
        tf.transformations.quaternion_matrix(q))
    
    converted = converted @ pose

    topush = Pose()
    topush.orientation = Quaternion(*tf.transformations.quaternion_from_matrix(converted))
    topush.position = Point(*tf.transformations.translation_from_matrix(converted))
    ground_truth.append([topush.position.x, topush.position.y])


R, t = icp(experiment, ground_truth)
print(R)
print(t)
transformed_ground_truth = np.dot(R, np.array(ground_truth).T) + t.reshape(-1, 1)
transformed_ground_truth = transformed_ground_truth.T

point_x = []
point_y = []
for pose in transformed_ground_truth:
    point_x.append(pose[0])
    point_y.append(pose[1])

ax.plot(point_x, point_y, 'r')

for hole in holes:
    hole_x = [point[0] for point in hole]
    hole_y = [point[1] for point in hole]
    hole_x.append(hole_x[0])  # Close the hole polygon
    hole_y.append(hole_y[0])  # Close the hole polygon
    plt.fill(hole_x, hole_y, 'grey')

ax.set_xlabel('X')
ax.set_ylabel('Y')

plt.show()

### 
gt = transformed_ground_truth.tolist()

print(len(gt))
experiment = random_discard(experiment, desired_length)

errors = compute_errors(experiment, gt)
write_to_file(errors, output_file)

x_err, y_err = compute_xy_errors(experiment, gt)
write_to_file(x_err, output_file_x)
write_to_file(y_err, output_file_y)