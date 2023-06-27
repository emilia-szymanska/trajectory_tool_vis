import matplotlib.pyplot as plt
import yaml

# Load the yaml file
with open('../sp_ws/src/trajectory_generator/trajectory_generation_tools/cfg/test_polygon_updated.yaml') as file:
    polygon_data = yaml.load(file, Loader=yaml.FullLoader)

with open('../sp_ws/src/trajectory_generator/trajectory_generation_tools/cfg/test_polygon.yaml') as file:
    initial_polygon_data = yaml.load(file, Loader=yaml.FullLoader)

# Extract polygon points
polygon_points = polygon_data['updated_polygon']['polygon']['hull']['points']
polygon_x = [point['x'] for point in polygon_points]
polygon_y = [point['y'] for point in polygon_points]

# Plot the polygon
plt.figure()
plt.fill(polygon_x, polygon_y, 'b', alpha=0.3)  # Blue color for the polygon

polygon_x.append(polygon_x[0])  # Close the hole polygon
polygon_y.append(polygon_y[0])
plt.plot(polygon_x, polygon_y, 'k')

# Plot the holes
for hole in polygon_data['updated_polygon']['polygon']['holes']:
    hole_points = hole['points']
    hole_x = [point['x'] for point in hole_points]
    hole_y = [point['y'] for point in hole_points]
    # plt.fill(hole_x, hole_y, 'w')
    hole_x.append(hole_x[0])  # Close the hole polygon
    hole_y.append(hole_y[0])  # Close the hole polygon
    plt.plot(hole_x, hole_y, 'r')  # White color for the holes

# Plot the holes
for hole in initial_polygon_data['polygon']['polygon']['holes']:
    hole_points = hole['points']
    hole_x = [point['x'] for point in hole_points]
    hole_y = [point['y'] for point in hole_points]
    hole_x.append(hole_x[0])  # Close the hole polygon
    hole_y.append(hole_y[0])  # Close the hole polygon
    plt.plot(hole_x, hole_y, 'k')  # White color for the holes

plt.show()