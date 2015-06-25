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
  
 Up Speed Control (using Barometer)

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """

from pid import PID
from scl import scl_get_socket, SCL_Reader
from msgpack import dumps, loads
from opcd_interface import OPCD_Subscriber
from geomath import deg2rad, sym_limit, angles_diff
from misc import daemonize


def main(name):
   pid = PID()
   opcd = OPCD_Subscriber()
   platform = opcd['platform']
   neutral_thrust = 9.81 * opcd[platform + '.mass']
   speed_setpoint = SCL_Reader('u_speed_ctrl', 'sub', -1.0)
   int_res = SCL_Reader('int_res', 'sub', 1)
   thrust_socket = scl_get_socket('thrust', 'pub')

   pos_speed_est = scl_get_socket('pos_speed_est_neu', 'sub')
   while True:
      u_speed = loads(pos_speed_est.recv())[3]
      err = speed_setpoint.data - u_speed
      pid.Kp = opcd['us_ctrl.p']
      pid.Ki = opcd['us_ctrl.i']
      thrust = neutral_thrust + pid.run(err)
      thrust_socket.send(dumps(thrust))


daemonize('u_spd_ctrl', main)
