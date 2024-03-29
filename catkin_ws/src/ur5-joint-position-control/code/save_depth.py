
#!/usr/bin/env python3

from ctypes import *
import os
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import time
import sensor_msgs.point_cloud2 as pc2


FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

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

def read_image(message):
    global count
    dir_path = os.path.dirname(os.path.realpath(__file__))
    voxel_size = 0.05

    data_path = '/../results/beer'

    pcd_files = []
    
    if(count<1):
        points = convertCloudFromRosToOpen3d(message)
        output_filename = os.path.abspath(dir_path + data_path) + f"/conversion_result_{count}_4.pcd"

        print(output_filename, points)

        o3d.io.write_point_cloud(output_filename, points)

        time.sleep(2)
        count+=1
    elif(count==1):
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
        
        print("numero punti in pcd:", len(pcd1.points))

        plane_model, inliers = pcd1.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations = 1000)
        pcd_without_plane = pcd1.select_by_index(inliers, invert=True)
        o3d.visualization.draw_geometries([pcd_without_plane])
        print("numero punti senza piano:", len(pcd_without_plane.points))
        count+=1

count = 0
rospy.init_node("read_image")
sub = rospy.Subscriber('/camera/depth/points', PointCloud2, read_image)
rospy.spin()


