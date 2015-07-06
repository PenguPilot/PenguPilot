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
  
 Flight State Detector based on N/E Accelerations

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from misc import daemonize
from scl import scl_get_socket
from msgpack import loads, dumps
from numpy import var


def main(name):
   acc_socket = scl_get_socket('acc_neu', 'sub')
   flying_socket = scl_get_socket('flying', 'pub')
   acc_vec = loads(acc_socket.recv())

   size = 80
   histn = [acc_vec[0]] * size
   histe = [acc_vec[1]] * size
   s = sp = 0

   i = 0
   while True:
      acc_vec = loads(acc_socket.recv())
      i += 1
      if i < 4:
         continue
      i = 0
      histn = histn[1:] + [acc_vec[0]]
      histe = histe[1:] + [acc_vec[1]]
      v = var(histn) + var(histe)
      if v > 15:
         s = 1
      else:
         s = 0
      if s != sp:
         flying_socket.send(dumps(s))
      sp = s

daemonize('flight_detect', main)
