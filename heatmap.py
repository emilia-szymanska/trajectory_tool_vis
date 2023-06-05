import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import box, Polygon
import numpy as np

def update_grid(grid, robot_width, robot_length, robot_position, robot_orientation):

    # Define robot as a rectangle
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

grid_cell_size = 0.1
grid_size = np.array([50.0, 50.0])  # size of the grid (width, height)
num_cells = (grid_size / grid_cell_size).astype(int)
grid = np.zeros(num_cells, dtype=int)

robot_width = 1.0  # width of the robot
robot_length = 2.0  # length of the robot
robot_position = np.array([5.0, 5.0])  # position of the robot (center of rectangle)
robot_orientation = np.pi/3  # orientation of the robot in radians (counter-clockwise from x-axis)

update_grid(grid, robot_width, robot_length, robot_position, robot_orientation)

plt.figure(figsize=(6,6)) # size of the figure
plt.imshow(grid, cmap='hot', interpolation='nearest')
plt.colorbar(label='Value') # Display a colorbar
plt.xlabel('X') # Label for the x-axis
plt.ylabel('Y') # Label for the y-axis
plt.title('Heatmap') # Title of the heatmap
plt.show()