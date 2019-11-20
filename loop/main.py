import numpy as np
import open3d as o3d
from PyNomaly import loop
import time
# from numba import jit

class PointSet:
    ply_=[]
    len_=[]
    tree_=[]
    def __init__( self, path):
      self.ply_ = o3d.io.read_point_cloud(path)
      self.len_=len(self.ply_.points)
      self.tree_=o3d.geometry.KDTreeFlann(self.ply_)      
      if len(self.ply_.colors)==0:
        self.ply_.paint_uniform_color([0,1.0,0])
    
    def write(self,path):
      o3d.io.write_point_cloud(path)

    def show(self):
      o3d.visualization.draw_geometries([self.ply_])


# @jit
def point_cloud_outlier_removal(ps):
  for i in range(ps.len_):
      [_, idx, dist] = ps.tree_.search_knn_vector_3d(ps.ply_.points[i], k)
      X=np.asarray(ps.ply_.points)[idx,:]
      m=loop.LocalOutlierProbability(X).fit()
      XScores=m.local_outlier_probabilities
      idx_outlier=np.where(XScores>0.8)[0]
      idx=np.asarray(idx)
      # for j in range(len(idx_outlier)):
      #     np.asarray(ps.ply_.colors)[idx[idx_outlier[j]],:]=[255,0,0]
      np.asarray(ps.ply_.colors)[idx[idx_outlier],:]=[255,0,0]


# Parameter
k=32
starttime = time.time()
# Read the point cloud
ps=PointSet("/home/llg/dataset/8_Couple.ply")
point_cloud_outlier_removal(ps)
# ps.write("llg-loop.ply")
endtime = time.time()
elapsed = endtime - starttime
print(elapsed)
ps.show()