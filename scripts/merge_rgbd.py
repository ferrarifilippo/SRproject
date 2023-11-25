import open3d as o3d
import numpy as np

def create_rgbd_image(rgb_file, depth_file):
    color_raw = o3d.io.read_image(rgb_file)
    depth_raw = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)
    return rgbd_image

def main():
    # Sostituisci con i percorsi dei tuoi file RGB e Depth
    rgb_file1 = "/home/filippo/PROJECT/src/ur5-jpc/results/rgb_image.png"
    depth_file1 = "/home/filippo/PROJECT/src/ur5-jpc/results/depth_image.png"
    
    rgb_file2 = "/home/filippo/PROJECT/src/ur5-jpc/results/rgb_image2.png"
    depth_file2 = "/home/filippo/PROJECT/src/ur5-jpc/results/depth_image2.png"

    # Crea le immagini RGBD da entrambe le coppie di file RGB e Depth
    rgbd_image1 = create_rgbd_image(rgb_file1, depth_file1)
    rgbd_image2 = create_rgbd_image(rgb_file2, depth_file2)

    # Creazione del PointCloud
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image1,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
        depth_scale=1.0/1000,
        depth_trunc=3.0,
        convert_rgb_to_intensity=False)

    pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image2,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
        depth_scale=1.0/1000,
        depth_trunc=3.0,
        convert_rgb_to_intensity=False)

    # Unione dei due PointCloud
    pcd1 += pcd2.transform(pcd1.get_rotation_matrix_from_xyz((0, np.pi, 0)))

    # Creazione della mesh dalla PointCloud
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd1, depth=8)

    # Visualizzazione della mesh
    o3d.visualization.draw_geometries([mesh])

if __name__ == "__main__":
    main()



