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
  
 Pilot Shell Setup Script

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
from pilot_pb2 import *
from pilot_interface import PilotInterface
from time import sleep
from math import sin, cos


_map = generate_map('pilot_shell')
_ctrl_socket = _map['ctrl']
_mon_socket = _map['mon']
i = PilotInterface(_ctrl_socket, _mon_socket)


rad = 5.0
pos = 0.0
while True:
   pos += 0.01
   i.set_ctrl_param(POS_N, cos(pos) * rad + rad)
   i.set_ctrl_param(POS_E, sin(pos) * rad + rad)
   sleep(0.2)

