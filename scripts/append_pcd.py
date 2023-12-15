import os
import open3d as o3d
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
    
data_path = '/../dataset/ambient/'

pcd_files = []

print(os.path.abspath(dir_path + data_path))

# Load point clouds
for root, dirs, files in os.walk(os.path.abspath(dir_path + data_path)):
    print(files)
    for file in files:
        print(file)
        pcd_files.append(os.path.join(root, file))

# Load fist point cloud
first_pcd = o3d.io.read_point_cloud(pcd_files[0])

# Height first point cloud
first_pcd_height = first_pcd.get_max_bound()[2]

# List of translated point cloud
translated_point_clouds = []

# Load the other point cloud, translated along the x-axis and apply preprocessing 
for i, pcd_file in enumerate(pcd_files[1:]):
    pcd = o3d.io.read_point_cloud(pcd_file)
    
    # Translates the current point cloud along the x-axis
    pcd.translate([i + 1, 0, 0])
    
    # Reduce the number of points in the current point cloud
    pcd = pcd.uniform_down_sample(every_k_points=100)  # Change the value of every_k_points to your liking
    
    # Add translocated and preprocessed point cloud to the list
    translated_point_clouds.append(pcd)

# Merge point cloud translocated into one
merged_point_cloud = o3d.geometry.PointCloud()
for pcd in [first_pcd] + translated_point_clouds:
    merged_point_cloud += pcd

# View the resulting point cloud
o3d.visualization.draw_geometries([merged_point_cloud])
