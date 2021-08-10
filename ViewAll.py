import numpy as np
import open3d as o3d
import sys, os

print(o3d.__version__)

filenames = []
for i in os.listdir(sys.argv[1]):
    if i.endswith(".ply"):
        filenames.append(i)

pcds = []
for i in filenames:
    pcd = o3d.io.read_point_cloud(sys.argv[1] + i)
    print(i, ": ", pcd)
    if len(pcd.points) < 1000:
        cl, ind = pcd.remove_radius_outlier(nb_points=20, radius=0.05)
        pcds.append(cl)
        o3d.visualization.draw_geometries([cl])
    else:
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=0.5)
        pcds.append(cl)
        o3d.visualization.draw_geometries([cl])

o3d.visualization.draw_geometries(pcds)
