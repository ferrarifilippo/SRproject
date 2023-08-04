import open3d as o3d
import numpy as np

def multiway_registration(pcds):
  """Performs multiway registration on a set of point clouds.

  Args:
    pcds: A list of point clouds.

  Returns:
    A list of point clouds, where each point cloud is aligned in the global
    space.
  """

  # Create a pose graph.
  pose_graph = o3d.PoseGraph()
  for i in range(len(pcds)):
    node = o3d.PoseGraphNode()
    node.set_points(pcds[i])
    node.T = np.identity(4)
    pose_graph.nodes.append(node)

  # Create edges between neighboring point clouds.
  for i in range(len(pcds)):
    for j in range(i + 1, len(pcds)):
      if i != j:
        edge = o3d.PoseGraphEdge()
        edge.source_node_id = i
        edge.target_node_id = j
        edge.transformation = o3d.registration.registration_icp(
            pcds[i], pcds[j], np.identity(4), max_correspondence_distance=0.02,
            max_iterations=100)
        pose_graph.edges.append(edge)

  # Optimize the pose graph.
  pose_graph.optimize(max_iterations=100)

  # Get the aligned point clouds.
  aligned_pcds = []
  for node in pose_graph.nodes:
    aligned_pcd = node.get_points()
    aligned_pcds.append(aligned_pcd)

  return aligned_pcds

if __name__ == "__main__":
  # Load the point clouds.

  pcds = []
  for i in range(3):
    pcd = o3d.io.read_point_cloud("pcd_%d.pcd" % i)
    pcds.append(pcd)

  # Perform multiway registration.
  aligned_pcds = multiway_registration(pcds)

  # Visualize the aligned point clouds.
  o3d.visualization.draw_geometries(aligned_pcds)
