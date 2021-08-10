import open3d as o3d
import sys

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud(sys.argv[1])
o3d.visualization.draw_geometries([pcd])

# print("Downsample the point cloud with a voxel of 0.02")
# voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.01)
# o3d.visualization.draw_geometries([voxel_down_pcd])

# print("Every 5th points are selected")
# uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
# o3d.visualization.draw_geometries([uni_down_pcd])

# print("Statistical oulier removal")
# cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.8)
# display_inlier_outlier(pcd, ind)
# o3d.visualization.draw_geometries([cl])

print("Radius oulier removal")
cl, ind = pcd.remove_radius_outlier(nb_points=10, radius=0.05)
display_inlier_outlier(pcd, ind)
o3d.visualization.draw_geometries([cl])