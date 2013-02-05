
import numpy as np


class QuadInvCoupling(object):

   std_matrix = np.array([
       # gas  pitch  roll   yaw
        [1,    0,    -1,    1],  # front motor (1)
        [1,    0,     1,    1],  # rear motor  (2)
        [1,   -1,     0,   -1],  # left motor  (3)
        [1,    1,     0,   -1]]) # right motor (4)

   def __init__(self, l, c, d):
      # l: rigger length
      # f = c * rpm ^ 2
      # tau = d * rpm ^ 2
      imtx1 = 1.0 / (4.0 * c)
      imtx2 = 1.0 / (2.0 * c * l)
      imtx3 = 1.0 / (4.0 * d)
      t = self.std_matrix.transpose()
      new_matrix = np.array([t[0] * imtx1, t[1] * imtx2, t[2] * imtx2, t[3] * imtx3])
      self.matrix = new_matrix.transpose()

