import matplotlib.pyplot as plt
import glob
import re

def plot_polygon(file_name):
    with open(file_name, 'r') as file:
        lines = file.readlines()

    x = []
    y = []
    linestyle = 'k-'  # initial linestyle for outer boundaries

    for line in lines:
        line = line.strip()

        # If the line is '-', it means we have finished a polygon
        if line == '-':
            plt.plot(x, y, linestyle)  # plot
            x = []
            y = []
            linestyle = 'k--'  # change linestyle to dashed for holes
        elif line == 'H':  # If the line is 'H', it means we have finished a hole
            linestyle = 'k-'  # change linestyle back to solid for the next outer boundary
        else:
            coords = re.findall(r"[\d\.\-e]+", line)
            x.append(float(coords[0]))
            y.append(float(coords[1]))

    # Plot the last polygon if there was no '-' after it
    if x and y:
        plt.plot(x, y, linestyle)

def main():
    plt.figure()

    # Loop over all .txt files in the current directory
    for file_name in glob.glob("polygon*.txt"):
        plot_polygon(file_name)

    plt.gca().set_aspect('equal', adjustable='box')  # to keep the aspect ratio
    plt.show()

if __name__ == "__main__":
    main()
