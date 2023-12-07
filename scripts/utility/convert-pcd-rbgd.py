import open3d as o3d
import numpy as np

# Load PCD file
pcd_path = "/Users/filippoferrari/Desktop/SRproject/dataset/beer/conversion_result_0.pcd"
pcd = o3d.io.read_point_cloud(pcd_path)

# Create a dummy RGB image and depth image for illustration purposes
# Replace these with your actual RGB and depth data
color = o3d.geometry.Image(np.array([[255, 0, 0]], dtype=np.uint8))  # Replace with your RGB image
depth = o3d.geometry.Image(np.array([[1.0]], dtype=np.float32))  # Replace with your depth image

# Convert RGB and depth images to RGBD image
rgbd_image = o3d.geometry.create_rgbd_image_from_color_and_depth(color, depth)

# Convert RGBD image to point cloud
rgbd_pointcloud = o3d.geometry.create_point_cloud_from_rgbd_image(rgbd_image,
                                                                  o3d.camera.PinholeCameraIntrinsic(
                                                                      o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

# Visualize the RGBD point cloud
o3d.visualization.draw_geometries([rgbd_pointcloud])