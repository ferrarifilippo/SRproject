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
    distance_threshold = 0.05
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500))
    return result

def main():

    dir_path = os.path.dirname(os.path.realpath(__file__))
    voxel_size = 0.005

    #data_path = '/Users/filippoferrari/Desktop/SRproject/catkin_ws/src/ur5-joint-position-control/results/beer'
    data_path = '/Users/filippoferrari/Desktop/SRproject/dataset/'

    pcd_files = []
    # Load point clouds
    for root, dirs, files in os.walk(os.path.abspath(data_path)):
        for file in files:
            if file.endswith('.pcd'):
                print(file)
                pcd_files.append(os.path.join(root, file))

    pcd1 = o3d.io.read_point_cloud(pcd_files[0])
    pcd1 = preprocess_point_cloud(pcd1, voxel_size)
    pcd_files.pop()

    # print("numero punti in pcd:", len(pcd1.points))
    #
    # plane_model, inliers = pcd1.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    # pcd_without_plane = pcd1.select_by_index(inliers, invert=True)
    # #o3d.visualization.draw_geometries([pcd_without_plane])
    # print("numero punti senza piano:", len(pcd_without_plane.points))
    #
    # pcd1 = pcd_without_plane

    for pcd_file in pcd_files:
        print(pcd_file)
        pcd2 = o3d.io.read_point_cloud(pcd_file)
        pcd2 = preprocess_point_cloud(pcd2, voxel_size)

        # print("numero punti in pcd:", len(pcd2.points))
        #
        # plane_model, inliers = pcd2.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        # pcd_without_plane = pcd2.select_by_index(inliers, invert=True)
        # # o3d.visualization.draw_geometries([pcd_without_plane])
        # print("numero punti senza piano:", len(pcd_without_plane.points))
        #
        # pcd2 = pcd_without_plane

        # Perform multiway registration
        trans12 = multiway_registration(pcd2, pcd1, np.identity(4))
        pcd2.transform(trans12.transformation)
        pcd1 = pcd1 + pcd2

    # print("numero punti in pcd:", len(pcd1.points))
    #
    # plane_model, inliers = pcd1.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    # pcd_without_plane = pcd1.select_by_index(inliers, invert=True)
    # o3d.visualization.draw_geometries([pcd_without_plane])
    # print("numero punti senza piano:", len(pcd_without_plane.points))

    # Visualize the merged point cloud
    o3d.visualization.draw_geometries([pcd1])

if __name__ == "__main__":
    main()
