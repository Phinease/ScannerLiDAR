import copy
import os
import sys

import numpy as np
import open3d as o3d


def pairwise_registration(source, target, mcdc, mcdf):
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]

        source_down = copy.deepcopy(source)
        target_down = copy.deepcopy(target)

        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=iter))
        current_transformation = result_icp.transformation

    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, mcdf, current_transformation)
    return current_transformation, information_icp


def full_registration(pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id], max_correspondence_distance_coarse, max_correspondence_distance_fine)
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(source_id, target_id,
                                                                                 transformation_icp, information_icp,
                                                                                 uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(source_id, target_id,
                                                                                 transformation_icp, information_icp,
                                                                                 uncertain=True))
    return pose_graph


def load_point_clouds():
    filenames = []
    for i in os.listdir(sys.argv[1]):
        if i.endswith(".ply"):
            filenames.append(i)

    nb_points = 0
    pcds = []
    for i in filenames:
        pcd = o3d.io.read_point_cloud(sys.argv[1] + i)

        if len(pcd.points) < 1000:
            pcd, _ = pcd.remove_radius_outlier(nb_points=20, radius=0.05)
        else:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=0.5)
        # o3d.visualization.draw_geometries([pcd])

        voxel_size = 0.02
        while len(pcd.points) > 1000:
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            voxel_size += 0.01

        if len(pcd.points) > 100:
            print(i, ": ", pcd)
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            pcds.append(pcd)
            nb_points += len(pcd.points)
            if nb_points > 10000:
                o3d.visualization.draw_geometries(pcds)
                return pcds

    o3d.visualization.draw_geometries(pcds)
    return pcds


print("Load point clouds ...")
voxel_size = 0.02
pcds_down = load_point_clouds()

print("Full registration ...")
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5
pose_graph = full_registration(pcds_down, max_correspondence_distance_coarse, max_correspondence_distance_fine)

print("Optimizing PoseGraph ...")
option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)
o3d.pipelines.registration.global_optimization(
    pose_graph,
    o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
    o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(), option)

print("Transform points and display ...")
pcd_combined = o3d.geometry.PointCloud()
for point_id in range(len(pcds_down)):
    print(pose_graph.nodes[point_id].pose)
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    cl, _ = pcds_down[point_id].remove_statistical_outlier(nb_neighbors=15, std_ratio=0.8)
    pcd_combined += cl

print("Complete")
o3d.visualization.draw_geometries([pcd_combined])
name = input("Output name: ")
o3d.io.write_point_cloud(name + ".ply", pcd_combined)
