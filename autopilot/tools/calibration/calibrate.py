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

 reads mag/acc log text file from stdin and writes
 calibration to stdout

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2012 Fabio Varesano <fabio at varesano dot net>

 Development of this code has been supported by the Department of Computer Science,
 Universita' degli Studi di Torino, Italy within the Piemonte Project
 http://www.piemonte.di.unito.it

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


def read_array():
   line = stdin.readline()
   if not line:
      return
   arr = line.split(' ')
   arr[5] = arr[5][0:-1]
   return arr


# read acc/mag data to array:
from sys import stdin
header = read_array()
if not header:
   raise ValueError("could not read txt file header")
mag_indices = []
for name in ['mag_x', 'mag_y', 'mag_z']:
   mag_indices.append(header.index(name))
acc_indices = []
for name in ['acc_x', 'acc_y', 'acc_z']:
   acc_indices.append(header.index(name))

from numpy import array
mag_list = []
acc_list = []
try:
   while True:
      msg = map(float, read_array())
      mag = []
      for i in mag_indices:
         mag.append(msg[i])
      mag_list.append(array(mag))
      acc = []
      for i in acc_indices:
         acc.append(msg[i])
      acc_list.append(array(acc))
except:
   pass

# compute calibration:
mag_cal = Calibration(mag_list)
acc_cal = Calibration(acc_list)

# verify calibration:
from math import isnan
from sys import exit
mag_cal_data = mag_cal.get_cal()
for val in mag_cal_data:
   if isnan(val):
      print 'calibration invalid'
      exit(1)
acc_cal_data = acc_cal.get_cal()
for val in acc_cal_data:
   if isnan(val):
      print 'calibration invalid'
      exit(2)

# print calibration to stdout:
cal_data = list(mag_cal_data) + list(acc_cal_data)
prefix = 'pilot.cal.'
i = 0
for sensor in ['mag', 'acc']:
   for type in ['bias', 'scale']:
      for axis in 'xyz':
         print prefix + sensor + '_' + type + '_' + axis, cal_data[i]
         i += 1


