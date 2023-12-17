import open3d as o3d
import numpy as np
import cv2

# Intel RealSense R200 camera parameters
fx = 451.984580
fy = 451.933982
cx = 0
cy = 0

depth_scaling_factor = 1000
img_center_x = cx
img_center_y = cy

img = cv2.imread('')
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
depth = o3d.io.read_image('')
depth = np.asarray(depth, np.float32)
threshold = 3000  # 1m limit
idx = np.where(depth > threshold)
depth[idx] = 0
print(depth.max(), depth.min())

original_pcd = o3d.geometry.PointCloud()
original_pcd_pos = []
original_pcd_color = []

for v in range(img.shape[0]):  # height
    for u in range(img.shape[1]):  # width
        z = depth[v][u] / depth_scaling_factor  # mm

        # Apply camera intrinsics to get 3D coordinates
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        original_pcd_pos.append([x, y, z])
        original_pcd_color.append(img[v][u] / 255)

original_pcd_pos = np.array(original_pcd_pos, dtype=np.float32)
original_pcd_color = np.array(original_pcd_color, dtype=np.float32)

original_pcd.points = o3d.utility.Vector3dVector(original_pcd_pos)
original_pcd.colors = o3d.utility.Vector3dVector(original_pcd_color)

# Save point cloud
o3d.io.write_point_cloud('', original_pcd)
o3d.visualization.draw_geometries([original_pcd])
