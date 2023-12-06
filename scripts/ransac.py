import os
import open3d as o3d

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0,
                                             max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0,
                                             max_nn=100))
    return (pcd_down, pcd_fpfh)

def register_all_pcd_files(folder_path, voxel_size=0.05, distance_multiplier=1.5,
                           max_iterations=1000000, confidence=0.999, mutual_filter=True):
    # Ottieni la lista dei file nella cartella
    pcd_files = [f for f in os.listdir(folder_path) if f.endswith('.pcd')]

    # Assicurati che ci siano almeno due file nella cartella
    if len(pcd_files) < 2:
        print("Ci devono essere almeno due file PCD nella cartella.")
        return

    # Usa il primo file come punto di riferimento
    reference_path = os.path.join(folder_path, pcd_files[0])
    reference_pcd = o3d.io.read_point_cloud(reference_path)

    # Inizializza la trasformazione cumulativa
    cumulative_transformation = o3d.pipelines.registration.PoseGraph()

    # Cicla su tutti gli altri file
    for j in range(1, len(pcd_files)):
        target_path = os.path.join(folder_path, pcd_files[j])
        target_pcd = o3d.io.read_point_cloud(target_path)

        print(f'Registering {reference_path} and {target_path}')

        # Esegui il downsampling
        reference_down, reference_fpfh = preprocess_point_cloud(reference_pcd, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target_pcd, voxel_size)

        # Esegui la registrazione
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            reference_down,
            target_down,
            reference_fpfh,
            target_fpfh,
            mutual_filter=mutual_filter,
            max_correspondence_distance=distance_multiplier * voxel_size,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_multiplier * voxel_size)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(max_iterations, confidence))

        # Aggiungi la trasformazione cumulativa
        cumulative_transformation.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(result.transformation))

    # Applica la trasformazione cumulativa al punto di riferimento iniziale
    o3d.pipelines.registration.global_optimization(cumulative_transformation)
    reference_pcd.transform(cumulative_transformation.nodes[0].pose)

    # Visualizza il risultato
    reference_pcd.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw([reference_pcd])
    o3d.io.write_point_cloud("risultato_registrazione_globale.pcd", reference_pcd)

if __name__ == '__main__':
    folder_path = '/Users/filippoferrari/Desktop/SRproject/dataset/glass/'
    register_all_pcd_files(folder_path)
