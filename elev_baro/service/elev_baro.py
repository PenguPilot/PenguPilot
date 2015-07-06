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
  
 Barometer Elevation based on Barometer Service

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from scl import scl_get_socket, SCL_Reader
from misc import daemonize
from msgpack import loads, dumps


def main(name):
   elev = SCL_Reader('elev', 'sub', None)
   baro_pos_speed_socket = scl_get_socket('pos_speed_est_neu', 'sub')
   elev_baro_socket = scl_get_socket('baro_elev', 'pub')
   start_pos = None
   while True:
      pos, speed = loads(baro_pos_speed_socket.recv())[2:4]
      if not start_pos:
         start_pos = pos
      if not elev.data:
         continue
      elev_baro_socket.send(dumps((pos - start_pos + elev.data, speed)))

daemonize('elev_baro', main)
