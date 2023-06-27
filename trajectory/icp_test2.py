import numpy as np
import csv
import matplotlib.pyplot as plt
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
import numpy as np

import numpy as np
from sklearn.neighbors import NearestNeighbors

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

experiment = []
for i, row in enumerate(rows):
    if i % 2 == 0 or i % 3 != 0:
        continue

    yaw = -row['ori_z']
    position = np.array([row['pos_x'], row['pos_y']])
    orientation = np.array([np.cos(yaw), np.sin(yaw)])
    
    # Normalize orientation to length 1
    orientation /= np.linalg.norm(orientation)

    ax.quiver(position[0], position[1], orientation[0], orientation[1], angles='xy', scale_units='xy', scale=1)
    experiment.append([row['pos_x'], row['pos_y']])


p0 = (rows[0]['pos_x'], rows[0]['pos_y'], 0.0)
q0 = (rows[0]['qx'], rows[0]['qy'], rows[0]['qz'], rows[0]['qw'])


#############################################
csvfile = open("../data/unsmoothed_multiple.csv")
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

ax.set_xlabel('X')
ax.set_ylabel('Y')

plt.show()