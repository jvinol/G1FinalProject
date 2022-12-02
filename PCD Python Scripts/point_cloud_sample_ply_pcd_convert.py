# convert ply file to pcd file

import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("apartment.ply") # Change this filename if using a different ply layout
o3d.io.write_point_cloud("apartment.pcd", pcd)
