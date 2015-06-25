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
  
 Up Speed Control

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
from msgpack import dumps, loads
from time import sleep
from opcd_interface import OPCD_Subscriber
from geomath import deg2rad, sym_limit, angles_diff
from misc import daemonize


def main(name):
   speed_setpoint = SCL_Reader('u_spd_ctrl', 'sub', -1.0)
   thrust_socket = scl_get_socket('thrust', 'pub')

   pos_speed_est = scl_get_socket('pos_speed_est_neu', 'sub')
   while True:
      u_speed = loads(pos_speed_est.recv())[3]
      err = speed_setpoint.data - u_speed
      print err
      thrust = 11.0 + err * 4.0
      thrust_socket.send(dumps(thrust))

main('')
daemonize('rp_ctrl', main)
