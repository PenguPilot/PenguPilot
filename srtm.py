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

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """



from osgeo import gdal
from osgeo.gdalconst import *
from numpy import matrix
import sys
import time
import os


class SrtmElevMap:

   def __init__(self, path):
      self.path = path
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
         print 'loading: ' + f
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
      if pixel[0] < 0.0 or pixel[0] > ds.RasterXSize or pixel[1] < 0.0 or pixel[1] > ds.RasterYSize:
         raise AssertionError('pixel out of bounds')

      # return elevation:
      return ds.ReadAsArray(int(pixel[0]), int(pixel[1]), 1, 1).ravel()[0]

from scl import generate_map
from gps_data_pb2 import GpsData
from os import getenv

socket = generate_map('gps_test')['gps']
m = SrtmElevMap(getenv('PENGUPILOT_PATH') + '/icarus/service/util/')
while True:
   gps_data = GpsData()
   gps_data.ParseFromString(socket.recv())
   if gps_data.fix == 3:
      print gps_data.alt




