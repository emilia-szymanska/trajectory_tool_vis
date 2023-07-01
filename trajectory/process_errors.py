import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import pandas as pd
from scipy.stats import norm


def read_from_file(filename):
    with open(filename, 'r') as file:
        data = [float(line.strip()) for line in file]
    return data

def plot_boxplot(data_unsmoothed, data_smoothed):
    # fig, ax = plt.subplots()
    # ax.boxplot([data_unsmoothed, data_smoothed], labels=["Unsmoothed trajectory", "Smoothed trajectory"])
    # ax.set_ylabel('Euclidian trajectory errors')
    # plt.show()

    fig, ax = plt.subplots()
    boxprops = dict(linestyle='-', linewidth=1.5, color='darkgoldenrod')
    flierprops = dict(marker='o', markerfacecolor='purple', markersize=6,
                  linestyle='none')
    medianprops = dict(linestyle='-', linewidth=2.5, color='firebrick')

    ax.boxplot([data_unsmoothed, data_smoothed], labels=["Unsmoothed trajectory", "Smoothed trajectory"], 
               patch_artist=True,  # fill with color
               boxprops=boxprops, flierprops=flierprops, medianprops=medianprops)
    
    colors = ['lightgreen', 'lightblue']
    
    # change color and linewidth of the whiskers
    for whisker in ax.lines[::5]:
        whisker.set(color='#8B008B', linewidth=1.5)

    # change color and linewidth of the caps
    for cap in ax.lines[1::5]:
        cap.set(color='#8B008B', linewidth=1.5)
        
    for patch, color in zip(ax.artists, colors):
        patch.set_facecolor(color)

    ax.set_ylabel('Euclidian trajectory errors [m]')
    plt.show()

def plot_6boxplots(data_x_unsmoothed, data_y_unsmoothed, data_euclid_unsmoothed, data_x_smoothed, data_y_smoothed, data_euclid_smoothed):
    fig, ax = plt.subplots()

    boxprops = dict(linestyle='-', linewidth=1.5, color='darkgoldenrod')
    flierprops = dict(marker='o', markerfacecolor='purple', markersize=6, linestyle='none')
    medianprops = dict(linestyle='-', linewidth=2.5, color='firebrick')

    box_data = [data_x_unsmoothed, data_y_unsmoothed, data_euclid_unsmoothed, data_x_smoothed, data_y_smoothed, data_euclid_smoothed]
    labels = ["Unsmoothed X", "Unsmoothed Y", "Unsmoothed Euclid", "Smoothed X", "Smoothed Y", "Smoothed Euclid"]

    ax.boxplot(box_data, labels=labels, patch_artist=True, whis=6.5, boxprops=boxprops, flierprops=flierprops, medianprops=medianprops)
    
    colors = ['lightgreen', 'lightgreen', 'lightgreen', 'lightblue', 'lightblue', 'lightblue']
    
    # change color and linewidth of the whiskers
    for whisker in ax.lines[::5]:
        whisker.set(color='#8B008B', linewidth=1.5)

    # change color and linewidth of the caps
    for cap in ax.lines[1::5]:
        cap.set(color='#8B008B', linewidth=1.5)
        
    for patch, color in zip(ax.artists, colors):
        patch.set_facecolor(color)

    ax.set_ylabel('Error Values [m]')
    plt.show()

def smooth_data(data, window_size=30):
    df = pd.DataFrame(data)
    smoothed_data = df.rolling(window=window_size, min_periods=1).mean()
    return smoothed_data.values

def print_percentiles(data_unsmoothed, data_smoothed):
    for name, data in zip(["Unsmoothed trajectory", "Smoothed trajectory"], [data_unsmoothed, data_smoothed]):
        print(f"{name}:")
        for perc in [1, 5, 10, 25, 50, 75, 90, 95, 99]:
            print(f"  {perc}th percentile: {np.percentile(data, perc):.2f}")

def plot_gaussians(data_unsmoothed, data_smoothed):
    fig, ax = plt.subplots()

    for name, data, color in zip(["Unsmoothed trajectory", "Smoothed trajectory"], [data_unsmoothed, data_smoothed], ['g', 'b']):
        mu, std = stats.norm.fit(data)
        xmin, xmax = plt.xlim()
        x = np.linspace(xmin, xmax, 100)
        p = stats.norm.pdf(x, mu, std)
        ax.plot(x, p, color=color, label=f"{name} (μ={mu:.2f}, σ={std:.2f})")

    ax.set_xlabel('Euclidian trajectory errors [m]')
    ax.set_ylabel('Probability density')
    plt.title('Gaussian fit of the trajectory errors')
    plt.legend()
    plt.show()

def plot_values(data_x, data_y, smoothed):
    plt.plot(data_x,   'k', label='x')
    plt.plot(data_y, 'r', label='y')
    plt.xlabel('Ids of consequitive poses')
    plt.ylabel('Distance error [m]')
    if smoothed == True:
        plt.title('Trajectory tracking error for smoothed trajectory')
    else:
        plt.title('Trajectory tracking error for unsmoothed trajectory')
    plt.legend(loc="upper left")
    plt.show()

# def plot_values(data_smoothed, data_unsmoothed):
#     plt.plot(data_smoothed,   'b', label='smoothed trajectory')
#     plt.plot(data_unsmoothed, 'g', label='unsmoothed trajectory')
#     plt.xlabel('Ids of consequitive poses')
#     plt.ylabel('Distance error')
#     plt.title('Trajectory tracking error')
#     plt.legend(loc="upper left")
#     plt.show()

def plot_histograms(data_unsmoothed, data_smoothed, bins=20):
    combined_data = np.concatenate((data_smoothed, data_unsmoothed))
    data_range = (np.min(combined_data), np.max(combined_data))
    plt.hist(data_unsmoothed, bins=bins, label="Unsmoothed trajectory", range=data_range, color="g")
    plt.hist(data_smoothed, bins=bins, label="Smoothed trajectory", alpha=0.5, range=data_range, color="b")

    avg_unsmoothed = np.mean(data_unsmoothed)
    avg_smoothed = np.mean(data_smoothed)
    plt.axvline(x=avg_unsmoothed, color='g', linestyle='dashed', linewidth=2, label=f'Avg unsmoothed: {avg_unsmoothed:.2f} m')
    plt.axvline(x=avg_smoothed, color='b', linestyle='dashed', linewidth=2, label=f'Avg smoothed: {avg_smoothed:.2f} m')
    
    plt.xlabel('Euclidian trajectory errors [m]')
    plt.ylabel('Frequency')
    plt.title('Trajectory errors distribution')
    plt.legend(loc='upper right')
    plt.show()


def read_from_file(filename):
    with open(filename, 'r') as file:
        data = [float(line.strip()) for line in file]
    return data


input_file_smoothed = "../data/errors_smoothed.csv"
input_file_unsmoothed = "../data/errors_unsmoothed.csv"

input_file_x_smoothed = "../data/errors_x_smoothed.csv"
input_file_x_unsmoothed = "../data/errors_x_unsmoothed.csv"

input_file_y_smoothed = "../data/errors_y_smoothed.csv"
input_file_y_unsmoothed = "../data/errors_y_unsmoothed.csv"


error_smoothed   = read_from_file(input_file_smoothed)
error_unsmoothed = read_from_file(input_file_unsmoothed)

error_x_smoothed   = read_from_file(input_file_x_smoothed)
error_x_unsmoothed = read_from_file(input_file_x_unsmoothed)

error_y_smoothed   = read_from_file(input_file_y_smoothed)
error_y_unsmoothed = read_from_file(input_file_y_unsmoothed)


print("=====")
print(f"Smoothed: {sum(error_smoothed)}, unsmoothed: {sum(error_unsmoothed)}")
print("======")
plot_6boxplots(error_x_unsmoothed, error_y_unsmoothed, error_unsmoothed, error_x_smoothed, error_y_smoothed, error_smoothed)

plot_histograms(error_unsmoothed, error_smoothed, bins=60)

error_x_smoothed = smooth_data(error_x_smoothed)
error_y_smoothed = smooth_data(error_y_smoothed)
error_x_unsmoothed = smooth_data(error_x_unsmoothed)
error_y_unsmoothed = smooth_data(error_y_unsmoothed)

# plot_values(error_x_smoothed, error_y_smoothed, smoothed=True)
# plot_values(error_x_unsmoothed, error_y_unsmoothed, smoothed=False)

data_unsmoothed = np.array(error_unsmoothed)
data_smoothed = np.array(error_smoothed)

# plot_boxplot(data_unsmoothed, data_smoothed)
print_percentiles(data_unsmoothed, data_smoothed)
plot_gaussians(data_unsmoothed, data_smoothed)