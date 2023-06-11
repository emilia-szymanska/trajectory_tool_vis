import matplotlib.pyplot as plt
import glob
import re

def plot_polygon(file_name):
    print(file_name)
    with open(file_name, 'r') as file:
        lines = file.readlines()

    x = []
    y = []

    for line in lines:
        line = line.strip()

        # If the line is '-', it means we have finished a polygon
        if line == '-':
            plt.plot(x, y, 'k-')  # plot in black
            x = []
            y = []
        else:
            coords = re.findall(r"[\d\.\-e]+", line)
            coords.pop(0)
            for i, el in enumerate(coords):
                if i%2 == 0:
                    y.append(float(el))
                else:
                    x.append(float(el))
            

    # Plot the last polygon if there was no '-' after it
    if x and y: 
        plt.plot(x, y, 'k-')

def main():
    plt.figure()

    # Loop over all .txt files in the current directory
    for file_name in glob.glob("polygon*.txt"):
        plot_polygon(file_name)

    plt.gca().set_aspect('equal', adjustable='box')  # to keep the aspect ratio
    plt.show()

if __name__ == "__main__":
    main()
