import open3d as o3d
import sys

bag_reader = o3d.t.io.RSBagReader()
bag_reader.open(sys.argv[1])
im_rgbd = bag_reader.next_frame()
while not bag_reader.is_eof():
    # process im_rgbd.depth and im_rgbd.color
    # TODO
    im_rgbd = bag_reader.next_frame()

bag_reader.close()