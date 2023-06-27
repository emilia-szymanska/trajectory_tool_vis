import numpy as np
import csv
import matplotlib.pyplot as plt
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
import numpy as np

def read_from_file(filename):
    with open(filename, 'r') as file:
        data = [float(line.strip()) for line in file]
    return data

def plot_values(data_smoothed, data_unsmoothed):
    """
    Plot the values from a list.

    Parameters:
    data (list): List of values.
    """
    plt.plot(data_smoothed,   'b', label='smoothed trajectory')
    plt.plot(data_unsmoothed, 'g', label='unsmoothed trajectory')
    plt.xlabel('Ids of consequitive poses')
    plt.ylabel('Distance error')
    plt.title('Trajectory tracking error')
    plt.legend(loc="upper left")
    plt.show()

def plot_histogram(data, bins=10, smoothed = 0):
    """
    Plot a histogram for a list of floating point values.

    Parameters:
    data (list): List of floating point values.
    bins (int): Number of bins in the histogram. Default is 10.
    """
    plt.hist(data, bins=bins)
    plt.xlabel('Euclidian trajectory errors')
    plt.ylabel('Frequency')
    if smoothed == 1:
        plt.title('Histogram of distance errors for smoothed trajectory')
    else:
        plt.title('Histogram of distance errors for unsmoothed trajectory')
    plt.show()

def read_from_file(filename):
    with open(filename, 'r') as file:
        data = [float(line.strip()) for line in file]
    return data


input_file_smoothed = "../data/errors_smoothed.csv"
input_file_unsmoothed = "../data/errors_unsmoothed.csv"

error_smoothed   = read_from_file(input_file_smoothed)
error_unsmoothed = read_from_file(input_file_unsmoothed)

# print(len(error_smoothed))
# print(len(error_unsmoothed))
print("=====")
print(f"Smoothed: {sum(error_smoothed)}, unsmoothed: {sum(error_unsmoothed)}")
print("======")

plot_histogram(error_smoothed, bins=20, smoothed=1)
plot_histogram(error_unsmoothed, bins=20, smoothed=0)

plot_values(error_smoothed, error_unsmoothed)