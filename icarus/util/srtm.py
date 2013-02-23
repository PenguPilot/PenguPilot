
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


from time import sleep
m = SrtmElevMap('./')
while True:
   sleep(0.01)
   print m.lookup((10.9142, 50.6872))

