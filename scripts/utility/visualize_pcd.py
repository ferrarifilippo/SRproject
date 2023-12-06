import open3d as o3d

pcd_file = ''

pcd = o3d.io.read_point_cloud(pcd_file)
o3d.visualization.draw(pcd)