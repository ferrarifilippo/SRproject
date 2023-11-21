import os
import open3d as o3d
import numpy as np

def downsample_point_cloud(pcd, voxel_size):
    return pcd.voxel_down_sample(voxel_size)

def preprocess_point_cloud(pcd, voxel_size):
    pcd = downsample_point_cloud(pcd, voxel_size)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=2 * voxel_size, max_nn=30))
    return pcd

def multiway_registration(source, target, trans_init):
    distance_threshold = 0.02
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
    return result

def main():

    dir_path = os.path.dirname(os.path.realpath(__file__))
    voxel_size = 0.05

    # Load point clouds
    pcd1 = o3d.io.read_point_cloud(dir_path + "/output.ply")
    pcd2 = o3d.io.read_point_cloud(dir_path + "output2.ply")
    #pcd3 = o3d.io.read_point_cloud("point_cloud_3.pcd")

    # Preprocess point clouds
    pcd1 = preprocess_point_cloud(pcd1, voxel_size)
    pcd2 = preprocess_point_cloud(pcd2, voxel_size)
    #pcd3 = preprocess_point_cloud(pcd3, voxel_size)

    # Perform multiway registration
    trans12 = multiway_registration(pcd2, pcd1, np.identity(4))
    pcd2.transform(trans12.transformation)

    #trans13 = multiway_registration(pcd3, pcd1, np.identity(4))
    #pcd3.transform(trans13.transformation)

    # Merge all point clouds
    final_pcd = pcd1 + pcd2 #+ pcd3

    # Visualize the merged point cloud
    o3d.visualization.draw_geometries([final_pcd])

if __name__ == "__main__":
    main()
