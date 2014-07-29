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
 
 SRTM Elevation Map

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from numpy import array
from osgeo import gdal
from osgeo.gdalconst import *
from numpy import matrix
import sys
import time
import os


def linfunc_from_points(v1, v2):
   m = (v2[1] - v1[1]) / (v2[0] - v1[0])
   n = v1[1] - m * v1[0]
   return m, n


def linfunc_calc(m, n, x):
   return float(m) * float(x) + float(n)


class SrtmElevMap:

   def __init__(self):
      self.path = os.path.join(os.path.dirname(__file__), '')
      self.cache = {}


   def _file_from_coord(self, coord):
      lon, lat = coord
      if lon >= 0.0: 
         lon_p = 'E'
      else: 
         lon_p = 'W'
      if lat >= 0.0: 
         lat_p = 'N'
      else: 
         lat_p = 'S'
      return self.path + lat_p + '%02d' % abs(lat) + lon_p + '%03d' % abs(lon) + '.hgt'


   def lookup(self, coord):
      f = self._file_from_coord(coord)
      try:
         # try to use file in cache:
         ds = self.cache[f]
      except:
         # load file:
         if not os.path.isfile(f):
            raise AssertionError('file does not exist: ' + f)
         ds = gdal.Open(f, GA_ReadOnly)
         if ds is None:
            raise AssertionError('could not load file')
         self.cache[f] = ds
      
      # Transform from map space to image space.
      world = matrix([[coord[0]], [coord[1]]])
      xform = ds.GetGeoTransform()
      offset = matrix([[xform[0]], [xform[3]]])
      A = matrix([[xform[1], xform[2]], [xform[4], xform[5]]])
      pixel = A.I * (world - offset)
      if pixel[0] < 0 or pixel[0] > ds.RasterXSize or pixel[1] < 0 or pixel[1] > ds.RasterYSize:
         raise AssertionError('pixel out of bounds')

      center_pixel = array([[round(pixel[0]), round(pixel[1])]]).T
      center_coord = A * center_pixel + offset
      center_elev = ds.ReadAsArray(int(center_pixel[0]), int(center_pixel[1]), 1, 1).ravel()[0]
      
      # left/right linear interpolation:
      left_pixel = array([center_pixel[0] - 1, center_pixel[1]])
      left_elev = ds.ReadAsArray(int(left_pixel[0]), int(left_pixel[1]), 1, 1).ravel()[0]
      left_coord = A * left_pixel + offset
      right_pixel = array([center_pixel[0] + 1, center_pixel[1]])
      right_elev = ds.ReadAsArray(int(right_pixel[0]), int(right_pixel[1]), 1, 1).ravel()[0]
      right_coord = A * right_pixel + offset
      if coord[0] < center_coord[0]:
         # left half
         m, n = linfunc_from_points([left_coord[0], left_elev], [center_coord[0], center_elev])
         x_elev = linfunc_calc(m, n, coord[0])
      elif coord[0] > center_coord[0]:
         # left half
         m, n = linfunc_from_points([center_coord[0], center_elev], [right_coord[0], right_elev])
         x_elev = linfunc_calc(m, n, coord[0])
      else:
         x_elev = center_elev
      
      # up/down linear interpolation:
      up_pixel = array([center_pixel[0], center_pixel[1] - 1])
      up_elev = ds.ReadAsArray(int(up_pixel[0]), int(up_pixel[1]), 1, 1).ravel()[0]
      up_coord = A * up_pixel + offset
      down_pixel = array([center_pixel[0], center_pixel[1] + 1])
      down_elev = ds.ReadAsArray(int(down_pixel[0]), int(down_pixel[1]), 1, 1).ravel()[0]
      down_coord = A * down_pixel + offset
      if coord[1] > center_coord[1]:
         # upper half
         m, n = linfunc_from_points([up_coord[1], up_elev], [center_coord[1], center_elev])
         y_elev = linfunc_calc(m, n, coord[1])
      elif coord[1] < center_coord[1]:
         # left half
         m, n = linfunc_from_points([center_coord[1], center_elev], [down_coord[1], down_elev])
         y_elev = linfunc_calc(m, n, coord[1])
      else:
         y_elev = center_elev

      # merge interpolations:
      return (x_elev + y_elev) / 2.0

