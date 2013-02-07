

import numpy
import quat


def inv_coupling_matrix_4(l, c, d):
   '''
   computes inverse coupling matrix with parameters:
      l = rigger length
      f = c * rpm ^ 2
      tau = d * rpm ^ 2
   '''
   std_coupling = numpy.array([
   # gas  pitch  roll   yaw
     [1,    0,    -1,    1],  # front motor (1)
     [1,    0,     1,    1],  # rear motor  (2)
     [1,   -1,     0,   -1],  # left motor  (3)
     [1,    1,     0,   -1]]) # right motor (4)
   imtx1 = 1.0 / (4.0 * c)
   imtx2 = 1.0 / (2.0 * c * l)
   imtx3 = 1.0 / (4.0 * d)
   t = std_coupling.transpose()
   new_matrix = numpy.array([t[0] * imtx1, t[1] * imtx2, t[2] * imtx2, t[3] * imtx3])
   return new_matrix.transpose()


def inv_coupling_matrix_to_config(matrix):
   class InvCoupling(object):
      pass
   coupling = InvCoupling()
   for (x, y), val in numpy.ndenumerate(matrix):
      setattr(coupling, '%d %d' % (x, y), val)
   return coupling


def vector_to_config(vec):
   class VecConf(object):
      pass
   vec_conf = VecConf()
   m = 'xyz'
   i = 0
   for val in vec:
      setattr(vec_conf, '%s' % m[i], val)
      i += 1
   return vec_conf



def marg_quats(*args, **kwargs):
   '''
   computes MARG sensor rotation quaternions
   '''
   marg_quat = kwargs.get('marg', quat(1))
   quats = {}
   for name in ['gyro', 'acc', 'mag']:
      sensor_quat = kwargs.get(name, quat(1))
      assert isinstance(sensor_quat, quat)
      quats[name] = marg_quat * sensor_quat
   return quats


