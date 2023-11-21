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

def main():
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # Paths to your RGB and depth images
    #rgb_path = dir_path + '/redwood-3dscan-master/data/rgbd/00001/rgb/0000001-000000000000.jpg'
    #depth_path = dir_path + '/redwood-3dscan-master/data/rgbd/00001/depth/0000001-000000000000.png'

    rgb_path = dir_path + '/redwood-3dscan-master/data/rgbd/00001/rgb/0000002-000000033516.jpg'
    depth_path = dir_path + '/redwood-3dscan-master/data/rgbd/00001/depth/0000002-000000033369.png'

    #dir_path + data_path + '/rgb/0000002-000000033516.jpg']
    #depth_files = [dir_path + data_path + '/depth/0000001-000000000000.png', dir_path + data_path + '/depth/0000002-000000033369.png']
    print(rgb_path)
    print(depth_path)
    # Replace these paths with your camera intrinsic and extrinsic files
    #intrinsic_path = "path/to/camera_intrinsic.json"
    #extrinsic_path = "path/to/camera_extrinsic.json"

    # Load camera intrinsic and extrinsic parameters
    #intrinsics = o3d.io.read_pinhole_camera_intrinsic(intrinsic_path)
    #extrinsics = o3d.io.read_pinhole_camera_parameters(extrinsic_path)

    # Create the complete point cloud from multiple views
    point_cloud = create_point_cloud(rgb_path, depth_path, o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))


    # Save the point cloud to a PLY file
    output_file = dir_path + "/" + "output.pcd"
    o3d.io.write_point_cloud(output_file, point_cloud)
    print("Point cloud saved to:", output_file)

if __name__ == "__main__":
    main()
