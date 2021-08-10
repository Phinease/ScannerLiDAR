import copy
import numpy as np
import open3d as o3d


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])



source = o3d.io.read_point_cloud("pointclouds/out1.ply")
source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
source = source.voxel_down_sample(voxel_size=0.02)
source, _ = source.remove_radius_outlier(nb_points=16, radius=0.05)
print("CHECK")
print("Load source done")

target = o3d.io.read_point_cloud("pointclouds/out2.ply")
target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
target = target.voxel_down_sample(voxel_size=0.02)
target, _ = target.remove_radius_outlier(nb_points=16, radius=0.05)
print("Load target done")

threshold = 0.1
trans_init = np.asarray([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

draw_registration_result(source, target, trans_init)

draw_registration_result_original_color(source, target, trans_init)
print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(
    source, target, threshold, trans_init)
print(evaluation)
# draw_registration_result(source, target, trans_init)

# reg_p2p = o3d.pipelines.registration.registration_icp(
#     source, target, threshold, trans_init,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
# print(reg_p2p)
# print("\nFinal alignment\nTransformation is:")
# print(reg_p2p.transformation)
# draw_registration_result(source, target, reg_p2p.transformation)

print("Apply point-to-plane ICP")
reg_p2l = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
print(reg_p2l)
print("Transformation is:")
print(reg_p2l.transformation)
draw_registration_result(source, target, reg_p2l.transformation)


# colored pointcloud registration
# This is implementation of following paper
# J. Park, Q.-Y. Zhou, V. Koltun,
# Colored Point Cloud Registration Revisited, ICCV 2017
# voxel_radius = [0.2, 0.04, 0.02, 0.01]
# max_iter = [100, 50, 30, 10]
# current_transformation = np.identity(4)
# for scale in range(4):
#     iter = max_iter[scale]
#     radius = voxel_radius[scale]
#
#     result_icp = o3d.pipelines.registration.registration_colored_icp(
#         source, target, radius, current_transformation,
#         o3d.pipelines.registration.TransformationEstimationForColoredICP(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
#                                                           relative_rmse=1e-6,
#                                                           max_iteration=iter))
#     current_transformation = result_icp.transformation
#     print(result_icp)
# draw_registration_result_original_color(source, target, result_icp.transformation)