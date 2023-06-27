import csv

file1 = "../data/smoothed3_gtsam_pose.csv"
file2 = "../data/smoothed31_gtsam_pose.csv"
file3 = '../data/smoothed_gtsam_pose.csv'

with open(file1, 'r') as file1:
    reader1 = csv.reader(file1, delimiter='\t')
    data1 = list(reader1)

data1 = data1[:-3300]

# Open the second file and read the lines into a list, excluding the first row (header) and the last 'lines_to_exclude' lines
with open(file2, 'r') as file2:
    reader2 = csv.reader(file2, delimiter='\t')
    data2 = list(reader2)

# data2 = data2[1:-700]
data2 = data2[600:-700]

# Combine the two lists
merged_data = data1 + data2

# Remove empty fields from each line in the merged data
merged_data = [[field for field in row if field] for row in merged_data]

# Write the merged data to a new csv file
with open(file3, 'w') as merged_file:
    writer = csv.writer(merged_file, delimiter='\t')
    writer.writerows(merged_data)

    