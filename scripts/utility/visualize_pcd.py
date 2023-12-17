import open3d as o3d
import numpy as np

# Function to rotate point cloud to align the plane with the horizontal axis
def align_plane_with_horizontal(pcd):
    # Estimate normals
    pcd.estimate_normals()

    # Take the average normal to represent the plane normal
    normal = np.asarray(pcd.normals).mean(axis=1)

    # Compute rotation matrix to align the plane with the horizontal axis
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((np.pi / 2 - np.arctan(normal[2] / normal[1]), 0, 0))

    # Apply rotation to point cloud
    pcd.rotate(rotation_matrix, center=(0, 0, 0))

# Animation callback function. It needs to contain as a minimum the visualizer reference
def rotate_around(vis):
    global pcd_without_plane
    # Rotate the point cloud
    R = o3d.geometry.get_rotation_matrix_from_xyz((0, np.deg2rad(0.5), 0))
    pcd_without_plane.rotate(R, center=(0, 0, 0))
    vis.update_geometry(pcd_without_plane)
    vis.update_renderer()


pcd_file = '/Users/filippoferrari/Desktop/SRproject/accumulated_mini_drone.pcd'

pcd = o3d.io.read_point_cloud(pcd_file)

# Ottieni le coordinate dei punti come un array NumPy
points = np.asarray(pcd.points)
theta = np.radians(-60)
R = np.array([[1, 0, 0],
              [0, np.cos(theta), -np.sin(theta)],
              [0, np.sin(theta), np.cos(theta)]])

# Applica la rotazione alla nuvola di punti
rotated_points = points.dot(R.T)
# Aggiorna le coordinate dei punti nella nuvola di punti
pcd.points = o3d.utility.Vector3dVector(rotated_points)

# Ottieni le coordinate dei punti come un array NumPy
points = np.asarray(pcd.points)
R = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
              [np.sin(np.pi), np.cos(np.pi), 0],
              [0, 0, 1]])

# Applica la rotazione alla nuvola di punti
rotated_points = points.dot(R.T)
# Aggiorna le coordinate dei punti nella nuvola di punti
pcd.points = o3d.utility.Vector3dVector(rotated_points)

#voxel_size = 0.0001
#pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
# o3d.visualization.draw_geometries([pcd])

vis = o3d.visualization.Visualizer()
vis.create_window('3DReconstructed')

# Align the plane with the horizontal axis
#align_plane_with_horizontal(pcd)

print("numero punti in pcd:", len(pcd.points))
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
pcd_without_plane = pcd.select_by_index(inliers, invert=True)
print("numero punti senza piano:", len(pcd_without_plane.points))

vis.add_geometry(pcd_without_plane)

# Register callback animation functions
vis.register_animation_callback(rotate_around)


vis.run()
vis.destroy_window()