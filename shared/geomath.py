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
 
 Geometric Functions Library

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


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


def limit(n, minn, maxn):
   return min(max(n, minn), maxn)


def sym_limit(n, m):
   return limit(n, -m, m)


def circle_err(b, a):
   d = b - a
   if d <= -pi:
      d += 2.0 * pi
   if d >= pi:
      d -= 2.0 * pi
   return d


def vec2_rot(v, angle):
   sa = sin(angle)
   ca = cos(angle)
   x = v[0] * ca - v[1] * sa
   y = v[0] * sa + v[1] * ca
   return x, y

