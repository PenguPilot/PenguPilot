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
  
 Pseudo GPS Sensor for Testing

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from random import random
from zmq import HWM
from time import sleep
from gps_data_pb2 import GpsData, SatInfo
from scl import generate_map


def main(name):
   socket = generate_map(name)['gps']
   socket.setsockopt(HWM, 1)
   while True:
      print 'send'
      gps_data = GpsData()
      sleep(0.2)
      gps_data.fix = 3
      gps_data.time = 'NONE'
      gps_data.lat = 50.0
      gps_data.lon = 10.0
      gps_data.alt = 500.0
      gps_data.sats = 6
      gps_data.hdop = 1.0
      gps_data.vdop = 2.0
      for i in range(0, 5):
         si = gps_data.satinfo.add()
         si.id = i
         si.in_use = int(random() + 0.5)
         si.elv = int(random() * 100) - 50
         si.azimuth = int(random() * 100) - 50
         si.sig = int(random() * 100)
      socket.send(gps_data.SerializeToString())


if __name__ == '__main__':
   main('gps_sensor')

