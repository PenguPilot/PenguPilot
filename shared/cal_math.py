#!/usr/bin/env python
"""
  ___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|

 3D Sensor ADC Calibration Math

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2013 Fabio Varesano <fabio at varesano dot net>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import numpy
from numpy import linalg


class Calibration(object):


   def __init__(self, init = None):
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
      self.sm = numpy.diag(self._cal[1] ** -1)


   def get_cal(self):
      return numpy.append(self._cal[0], self._cal[1])


   def _calibrate(self, x, y, z):
      import warnings
      with warnings.catch_warnings():
         warnings.simplefilter("ignore")
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

