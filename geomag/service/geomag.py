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
  
 Geomag Declination Lookup Service

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from geomag.geomag import GeoMag
from scl import scl_get_socket
from misc import daemonize, RateTimer
import os
from time import time
from datetime import datetime
from gps_msgpack import *
from msgpack import loads, dumps
from math import pi


def main(name):
   script_path = os.path.dirname(os.path.abspath(__file__))
   gm = GeoMag(script_path + os.sep + 'geomag' + os.sep + 'WMM.COF')
   gps_socket = scl_get_socket('gps', 'sub')
   decl_socket = scl_get_socket('decl', 'pub')
   rt = RateTimer(1)
   while True:
      raw = gps_socket.recv()
      if rt.expired():
         gps = loads(raw)
         try:
            date = datetime.strptime(gps[TIME], '%Y-%m-%d %H:%M:%S').date()
            decl = pi * gm.GeoMag(gps[LAT], gps[LON], time = date).dec / 180.0
            decl_socket.send(dumps(decl))
         except:
            pass

daemonize('geomag', main)
