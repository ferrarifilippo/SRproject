import os
import numpy as np
import open3d as o3d

def load_rgbd_image(rgb_file, depth_file):
    # Load RGB image and depth image
    rgb = o3d.io.read_image(rgb_file)
    depth = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth)
    return rgbd_image

#def create_point_cloud(rgb_files, depth_files, intrinsics, extrinsics):
def create_point_cloud(rgb_file, depth_file, intrinsics):
    point_cloud = o3d.geometry.PointCloud()
    
    # Load the RGBD image
    rgbd_image = load_rgbd_image(rgb_file, depth_file)

    # Convert the RGBD image to a point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)

    return pcd

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

    data_path = '/redwood-3dscan-master/data/rgbd/00001'

    rgb_files = []
    depth_files = []

    i = 0
    for root, dirs, files in os.walk(os.path.abspath(dir_path + data_path + '/rgb/')):
        for file in files:
            if(i<10):
                rgb_files.append(os.path.join(root, file))
                i += 1

    j = 0
    for root, dirs, files in os.walk(os.path.abspath(dir_path + data_path + '/depth/')):
        for file in files:
            if j<=i:
                depth_files.append(os.path.join(root, file))
            j += 1
        

    # Camera intrinsic parameters (fx, fy, cx, cy) obtained from camera calibration
    fx = 500.0
    fy = 500.0
    cx = 320.0
    cy = 240.0
    intrinsics = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0, 0, 1]])

    for j in range(i): 
    
        point_cloud = create_point_cloud(rgb_files[j], depth_files[j], o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

        point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        #o3d.visualization.draw_geometries([point_cloud])

        # Save the point cloud to a PLY file
        filename = "output{0}.ply".format(j)
        output_file = dir_path + "/" + filename
        o3d.io.write_point_cloud(output_file, point_cloud)
        print("Point cloud saved to:", output_file)

    voxel_size = 0.005

    pcd_initial = o3d.io.read_point_cloud(dir_path + "/output0.ply")
    pcd_initial = preprocess_point_cloud(pcd_initial, voxel_size)

    final_pcd = pcd_initial

    for j in range(i):

        if j == 0:
            continue
        # Load point clouds
        filename = "output{0}.ply".format(j)
        pcd = o3d.io.read_point_cloud(dir_path + "/" + filename)

        # Preprocess point clouds
        pcd = preprocess_point_cloud(pcd, voxel_size)
        #pcd3 = preprocess_point_cloud(pcd3, voxel_size)

        # Perform multiway registration
        trans= multiway_registration(pcd, pcd_initial, np.identity(4))
        pcd.transform(trans.transformation)

        # Merge all point clouds
        final_pcd += pcd 

    # Visualize the merged point cloud
    o3d.visualization.draw_geometries([final_pcd])

if __name__ == "__main__":
    main()