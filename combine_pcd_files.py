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

    data_path = '/cougarbot/results/'

    pcd_files = []
    # Load point clouds
    for root, dirs, files in os.walk(os.path.abspath(dir_path + data_path)):
        for file in files:
            print(file)
            pcd_files.append(os.path.join(root, file))

    pcd1 = o3d.io.read_point_cloud(pcd_files[0])
    pcd1 = preprocess_point_cloud(pcd1, voxel_size)
    pcd_files.pop()

    for pcd_file in pcd_files:
        pcd2 = o3d.io.read_point_cloud(pcd_file)
        # Perform multiway registration
        trans12 = multiway_registration(pcd2, pcd1, np.identity(4))
        pcd2.transform(trans12.transformation)
        pcd1 = pcd1 + pcd2

    # Preprocess point clouds
    #pcd1 = preprocess_point_cloud(pcd1, voxel_size)
    #pcd2 = preprocess_point_cloud(pcd2, voxel_size)
    #pcd3 = preprocess_point_cloud(pcd3, voxel_size)

    # Perform multiway registration
    #trans12 = multiway_registration(pcd2, pcd1, np.identity(4))
    #pcd2.transform(trans12.transformation)

    #trans13 = multiway_registration(pcd3, pcd1, np.identity(4))
    #pcd3.transform(trans13.transformation)

    # Merge all point clouds
    #final_pcd = pcd1 + pcd2 #+ pcd3

    # Visualize the merged point cloud
    o3d.visualization.draw_geometries([pcd1])

if __name__ == "__main__":
    main()
