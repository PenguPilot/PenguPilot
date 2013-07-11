

#
# file: geomath.py
# purpose: various geodetic math functions
#
# authors: Tobias Simon, Ilmenau University of Technology
#          Jan Roemisch, Ilmenau University of Technology
#


from math import sin, cos, acos, atan2, pi, fmod, hypot, asin
from pyproj import Proj, transform


def deg2rad(deg):
   return deg / 180.0 * pi


def rad2deg(rad):
   return rad * 180.0 / pi


def bearing(lat1, lon1, lat2, lon2):
   dLon = lon2 - lon1
   y = sin(dLon) * cos(lat2)
   x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
   return atan2(y, x)


def bearing_deg(*args): # lat1, lon1, lat2, lon2
   args = map(deg2rad, args)
   return rad2deg(bearing(*args))


class LinearInterpolation:

   def __init__(self, x1, y1, x2, y2):
      self.m = (float(y2) - float(y1)) / (float(x2) - float(x1))
      self.n = float(y1) - self.m * float(x1)

   def calc(self, x):
      return self.m * float(x) + self.n


_gps = Proj(proj = 'latlong', datum = 'WGS84')
_aeqd = Proj(proj = 'aeqd')


def gps_add_meters((lat, lon), (dx, dy)):
   y, x = transform(_gps, _aeqd, lat, lon)
   y += dy; x += dx
   return transform(_aeqd, _gps, y, x)


def gps_meters_offset((lat1, lon1), (lat2, lon2)):
   y1, x1 = transform(_gps, _aeqd, lat1, lon1)
   y2, x2 = transform(_gps, _aeqd, lat2, lon2)
   return x2 - x1, y2 - y1



if __name__ == '__main__':

   import unittest

   class TestGeoMath(unittest.TestCase):

      def test_bearing(self):
         self.assertEqual(bearing_deg(50.0, 5.0, 55.0, 7.0), 12.896821904227854)
         self.assertEqual(bearing_deg(50.0, 5.0, 50.0, 7.0), 89.23392341814612)

      def test_transformations(self):
         self.assertEqual(gps_add_meters(50.0, 10.0, 1000000, 0), (51.292293466969085, 17.86742449270132))
         self.assertEqual(gps_meters_offset(0, 0, 0.1, 1), (110574.3885577996, 11130.830075575039))

   unittest.main()

