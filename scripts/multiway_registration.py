# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------
"""Align multiple pieces of geometry in a global space"""

import open3d as o3d
import numpy as np
import os


def load_point_clouds(voxel_size=0.0):

    data_path = '/Users/filippoferrari/Desktop/SRproject/beer_new'

    pcd_files = []
    # Load point clouds
    for root, dirs, files in os.walk(os.path.abspath(data_path)):
        for file in sorted(files):
            if file.endswith('.pcd'):
                print(file)
                pcd_files.append(os.path.join(root, file))

    pcd_files.reverse()
    pcds = []
    for pcd_file in pcd_files:
        print(pcd_file)
        pcd = o3d.io.read_point_cloud(pcd_file)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)

    return pcds
    
def segment_and_align_planes(source, target, distance_threshold=0.01):
    # Segment planes
    source_plane, source_inliers = source.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)
    target_plane, target_inliers = target.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)

    # Extract planes
    # source_plane = source.extract_geometry(source_inliers, invert=True)
    # target_plane = target.extract_geometry(target_inliers, invert=True)

    # Align planes
    plane_transformation, _ = o3d.pipelines.registration.registration_icp(
        source_plane, target_plane, 0.1, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    
    # Transform the entire source PointCloud
    source.transform(plane_transformation)

    return source, plane_transformation

def pairwise_registration(source, target, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
    print("Estimating normals for source and target")
    source.estimate_normals()
    target.estimate_normals()

    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)

    # Align planes
    source_aligned, plane_transformation = segment_and_align_planes(source, target)

    # Combine the transformations
    transformation_combined = np.dot(transformation_icp, plane_transformation)

    return source_aligned, target, transformation_combined, information_icp

def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            source_aligned, target_aligned, transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],
                max_correspondence_distance_coarse,
                max_correspondence_distance_fine)
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph


if __name__ == "__main__":
    voxel_size = 0.001
    pcds_down = load_point_clouds(voxel_size)
    o3d.visualization.draw(pcds_down)

    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds_down,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    o3d.visualization.draw(pcds_down)