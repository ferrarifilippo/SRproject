import os
import cv2
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt

# Function to load RGBD images (provide paths to your RGB and depth images)
def load_rgbd_image(rgb_path, depth_path):
    rgb = cv2.imread(rgb_path)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0  # Scale depth values (depends on sensor)
    return rgb, depth

# Function to create a point cloud from RGBD images
def create_point_cloud(rgb, depth, intrinsic_matrix):
    h, w = depth.shape
    v, u = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    points_2d = np.stack((u.flatten(), v.flatten(), np.ones(u.size)), axis=-1)
    points_3d = np.dot(np.linalg.inv(intrinsic_matrix), points_2d.T * depth.flatten()).T
    colors = rgb.reshape((-1, 3))
    return points_3d, colors

# Main function
def main():
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # Paths to your RGB and depth images
    rgb_path = dir_path + '/redwood-3dscan-master/data/rgbd/00001/rgb/0000001-000000000000.jpg'
    depth_path = dir_path + '/redwood-3dscan-master/data/rgbd/00001/depth/0000001-000000000000.png'

    # Camera intrinsic parameters (fx, fy, cx, cy) obtained from camera calibration
    fx = 500.0
    fy = 500.0
    cx = 320.0
    cy = 240.0
    intrinsic_matrix = np.array([[fx, 0, cx],
                                 [0, fy, cy],
                                 [0, 0, 1]])

    color_raw = o3d.io.read_image(rgb_path)
    depth_raw = o3d.io.read_image(depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw)
    print(rgbd_image)

    #plt.subplot(1, 2, 1)
    #plt.title('Redwood grayscale image')
    #plt.imshow(rgbd_image.color)
    #plt.subplot(1, 2, 2)
    #plt.title('Redwood depth image')
    #plt.imshow(rgbd_image.depth)
    #plt.show()

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])

    # Load RGBD images
    #rgb_image, depth_image = load_rgbd_image(rgb_path, depth_path)

    # Create point cloud
    #points_3d, colors = create_point_cloud(rgb_image, depth_image, intrinsic_matrix)

    # Create Open3D point cloud object
    #point_cloud = o3d.geometry.PointCloud()
    #point_cloud.points = o3d.utility.Vector3dVector(points_3d)
    #point_cloud.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    #o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    main()
