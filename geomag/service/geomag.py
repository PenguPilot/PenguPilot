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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from geomag.geomag import GeoMag
from scl import generate_map
from gps_data_pb2 import GpsData
from misc import daemonize
import os
from time import time
from datetime import datetime


def main(name):
   script_path = os.path.dirname(os.path.abspath(__file__))
   gm = GeoMag(script_path + os.sep + 'geomag' + os.sep + 'WMM.COF')
   socket_map = generate_map(name)
   gps_socket = socket_map['gps']
   decl_socket = socket_map['decl']
   i = 0
   while True:
      data = gps_socket.recv()
      if i == 20:
         i = 0
         gps_data = GpsData()
         gps_data.ParseFromString(data)
         if gps_data.time and gps_data.fix >= 2:
            date = datetime.strptime(gps_data.time, '%Y-%m-%d %H:%M:%S').date()
            decl = gm.GeoMag(gps_data.lat, gps_data.lon, time = date).dec
            print time(), decl
            decl_socket.send('%f' % decl)
      i += 1

daemonize('geomag', main)
