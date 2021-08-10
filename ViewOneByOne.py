import numpy as np
import open3d as o3d
import sys, os

print(o3d.__version__)

filenames = []
for i in os.listdir(sys.argv[1]):
    if i.endswith(".ply"):
        filenames.append(i)

for i in filenames:
    pcd = o3d.io.read_point_cloud(sys.argv[1] + i)
    print(i, ": ", pcd)
    o3d.visualization.draw_geometries([pcd])