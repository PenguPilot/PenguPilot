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
  
 GPX Logger

 Copyright (C) 2011 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import sys
import zmq
from gps_data_pb2 import GpsData
from gpxdata import TrackPoint, TrackSegment, Track, Document
from zmq_ipc import generate_map


socket = generate_map('gpx_logger')['gps']
gps_data = GpsData()
segment = TrackSegment()
try:
   while True:
      str = socket.recv()
      gps_data.ParseFromString(str)
      print gps_data
      point = TrackPoint(gps_data.lat, gps_data.lon, gps_data.alt)
      segment.appendPoint(point)
except:
   pass

track = Track(name = "Copter Track", description = "Track recorded using the copter's onboard GPS module")
track.appendSegment(segment)
doc = Document([track], name = "Copter GPX Document")
doc.writeGPX(sys.stdout)

