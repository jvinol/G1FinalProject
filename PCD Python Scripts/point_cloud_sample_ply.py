import numpy as np
import pyvista as pv

point_cloud = pv.read('apartment.ply')
point_cloud.plot(eye_dome_lighting=True, window_size=[2000,1500])
