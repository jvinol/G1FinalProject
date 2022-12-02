# Plot .pcd file colored by elevation

import numpy as np
import open3d as o3d
import pyvista as pv

pcd = o3d.io.read_point_cloud('apartment.pcd') # Change this filename to the filename used to save the data obtained from point_cloud_sample_ply_pcd_convert.py
points_list = np.asarray(pcd.points)  
print("output array from input list : ", points_list)
print("Number of points : ", len(points_list))

point_cloud = pv.PolyData(points_list)

data = points_list[:, -1]
point_cloud['elevation'] = data

point_cloud.plot(eye_dome_lighting=False, window_size=[2000,1500])
