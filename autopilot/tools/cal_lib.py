"""

Ellipsoid into Sphere calibration library based upon numpy and linalg
Copyright (C) 2012 Fabio Varesano <fabio at varesano dot net>
Copyright (C) 2013 Tobias Simon <tobias.simon@tu-ilmenau.de>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""


import numpy
from numpy import linalg


class Calibration(object):


   def __init__(self, init = None):
      #if 1:
      try:
         if isinstance(init, list):
            if len(init) == 6:
               # calibration given in values:
               self._cal = numpy.array(init[0:3]), numpy.array(init[3:6])
            else:
               x, y, z = numpy.array(init).T
               self._cal = self._calibrate(x, y, z)
         elif isinstance(init, str):
            # calibrate from file:
            x, y, z = numpy.loadtxt(init).T
            self._cal = self._calibrate(x, y, z)
         else:
            # use stdin:
            from sys import stdin
            x, y, z = numpy.loadtxt(stdin).T
            self._cal = self._calibrate(x, y, z)
      except:
         raise ValueError('expected filename or 6-element float array')
      self.sm = numpy.diag(self._cal[1] ** -1)


   def get_cal(self):
      return numpy.append(self._cal[0], self._cal[1])


   def _calibrate(self, x, y, z):
      # prepare arrays:
      H = numpy.array([x, y, z, -y ** 2, -z ** 2, numpy.ones([len(x), 1])]).T
      w = x ** 2
      # solve least-squares problem:
      (X, residues, rank, shape) = linalg.lstsq(H, w)
      # compute offsets:
      ox = X[0] / 2
      oy = X[1] / (2 * X[3])
      oz = X[2] / (2 * X[4])
      # compute helper values:
      A = X[5] + ox ** 2 + X[3] * oy ** 2 + X[4] * oz ** 2
      B = A / X[3]
      C = A / X[4]
      # compute scales:
      sx = numpy.sqrt(A)
      sy = numpy.sqrt(B)
      sz = numpy.sqrt(C)
      # return offset and scale arrays
      return numpy.array([ox, oy, oz]), numpy.array([sx, sy, sz])


   def apply(self, vec):
      return (self.sm * (vec - self._cal[0])).diagonal()


   def apply_array(self, data):
      adj_data = []
      for vec in data:
         adj_data.append(self.apply(vec))
      return numpy.array(adj_data)
