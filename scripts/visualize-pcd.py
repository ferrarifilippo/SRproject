import open3d as o3d

pcd_file = '/Users/filippoferrari/Downloads/3D_Object_Reconstruction-main/results/accumulated_spyderman2.pcd'

pcd = o3d.io.read_point_cloud(pcd_file)
o3d.visualization.draw(pcd)