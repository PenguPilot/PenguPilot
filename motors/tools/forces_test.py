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
  
 OMAP3-PWM Motor Test Program

 Copyright (C) 2014 Tobias Simon

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """



from time import sleep
from sys import argv
from scl import scl_get_socket
from msgpack import dumps

socket = scl_get_socket('motor_forces', 'pub')

def write_motors(en, val):
   forces = [en, val, val, val, val]
   socket.send(dumps(forces))


while 1:
   for _ in range(2000):
      write_motors(1, 0.0)
      sleep(0.005)
   for _ in range(2000):
      write_motors(0, 0.0)
      sleep(0.005)

