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
  
 Remote Control Calibrated Channels Dumper

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from scl import generate_map
from msgpack import loads
from misc import RateTimer

try:
   socket_map = generate_map('rc_cal_dump')
   s = socket_map['rc_cal']
   rt = RateTimer(2.0)
   while True:
      data = loads(s.recv())
      valid = bool(data[0])
      c = data[1:]
      if valid:
         print 'pitch: %.1f\troll: %.1f\tyaw: %.1f\tgas: %.1f\ttwo_state: %.1f\tthree_state: %.1f' % (c[0], c[1], c[2], c[3], c[4], c[5])
      elif rt.expired():
         print 'signal invalid'
except:
   print 'canceled by user'

