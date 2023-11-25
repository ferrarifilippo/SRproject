import os
import open3d as o3d
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
    
data_path = '/../results/ambient'

pcd_files = []

# Load point clouds
for root, dirs, files in os.walk(os.path.abspath(dir_path + data_path)):
    for file in files:
        print(file)
        pcd_files.append(os.path.join(root, file))

# Carica la prima point cloud
first_pcd = o3d.io.read_point_cloud(pcd_files[0])

# Altezza della prima point cloud
first_pcd_height = first_pcd.get_max_bound()[2]

# Lista per memorizzare le singole point cloud traslate
translated_point_clouds = []

# Carica le altre point cloud, traslale lungo l'asse x e applica il preprocessing
for i, pcd_file in enumerate(pcd_files[1:]):
    pcd = o3d.io.read_point_cloud(pcd_file)
    
    # Trasla la point cloud corrente lungo l'asse x
    pcd.translate([i + 1, 0, 0])
    
    # Riduci il numero di punti nella point cloud corrente
    pcd = pcd.uniform_down_sample(every_k_points=100)  # Cambia il valore di every_k_points a tuo piacimento
    
    # Aggiungi la point cloud traslata e preprocessata alla lista
    translated_point_clouds.append(pcd)

# Unisci le point cloud traslate in una sola
merged_point_cloud = o3d.geometry.PointCloud()
for pcd in [first_pcd] + translated_point_clouds:
    merged_point_cloud += pcd

# Visualizza la point cloud risultante
o3d.visualization.draw_geometries([merged_point_cloud])
