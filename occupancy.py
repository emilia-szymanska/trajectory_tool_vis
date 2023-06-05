import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import box, Polygon
import numpy as np

# Assuming robot parameters
robot_width = 1.0  # width of the robot
robot_length = 2.0  # length of the robot
robot_position = np.array([5.0, 5.0])  # position of the robot (center of rectangle)
robot_orientation = np.pi/3  # orientation of the robot in radians (counter-clockwise from x-axis)

# Assuming grid parameters
grid_cell_size = 0.1  # size of a grid cell
grid_size = np.array([10.0, 10.0])  # size of the grid (width, height)

# Calculate number of cells in each direction
num_cells = (grid_size / grid_cell_size).astype(int)

# Initialize grid as free
grid = np.zeros(num_cells, dtype=int)

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

# Check each cell if it is intersecting with the robot
for i in range(num_cells[0]):
    for j in range(num_cells[1]):
        cell_corners = np.array([
            [i*grid_cell_size, j*grid_cell_size],
            [(i+1)*grid_cell_size, j*grid_cell_size],
            [(i+1)*grid_cell_size, (j+1)*grid_cell_size],
            [i*grid_cell_size, (j+1)*grid_cell_size]
        ])
        cell = Polygon(cell_corners)
        if robot.intersects(cell):
            grid[j, i] += 1

# Print indices of occupied cells
# occupied_indices = np.transpose(np.nonzero(grid))

# Assuming you have a 2D numpy array called 'grid' with your data

plt.figure(figsize=(6,6)) # size of the figure
plt.imshow(grid, cmap='hot', interpolation='nearest')
plt.colorbar(label='Value') # Display a colorbar
plt.xlabel('X') # Label for the x-axis
plt.ylabel('Y') # Label for the y-axis
plt.title('Heatmap') # Title of the heatmap
plt.show()