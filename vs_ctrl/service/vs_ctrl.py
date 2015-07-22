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
  
 Vertical Speed Control (using Barometer)

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from pid_ctrl import PID_Ctrl
from scl import scl_get_socket, SCL_Reader
from opcd_interface import OPCD_Subscriber
from physics import G_CONSTANT
from misc import daemonize
from pp_prio import PP_PRIO_3
from scheduler import sched_set_prio
from mot_state import STOPPED, RUNNING


def main(name):
   sched_set_prio(PP_PRIO_3)
   pid = PID_Ctrl()
   opcd = OPCD_Subscriber()
   platform = opcd['platform']
   neutral_thrust = G_CONSTANT * opcd[platform + '.mass']
   thrust_max = opcd[platform + '.thrust_max']
   speed_setpoint = SCL_Reader(name + '_sp', 'sub', -1.0)
   oe = SCL_Reader(name + '_oe', 'pull', 1)
   err = scl_get_socket(name + '_err', 'pub')
   ms = SCL_Reader('mot_state', 'sub', STOPPED)
   thrust_socket = scl_get_socket('thrust_p', 'push')

   pos_speed_est = scl_get_socket('pos_speed_est_neu', 'sub')
   while True:
      u_speed = pos_speed_est.recv()[3]
      max_speed = opcd[name + '.max_speed']
      pid.p = opcd[name + '.p']
      pid.i = opcd[name + '.i']
      pid.max_sum_err = opcd[name + '.max_sum_err']
      err.send(pid.err)
      if ms.data != RUNNING or not oe.data:
         pid.reset()
      thrust = neutral_thrust + pid.control(u_speed, min(max_speed, speed_setpoint.data))
      pid.int_en = thrust < 0.0 or thrust > thrust_max
      if oe.data:
         thrust_socket.send(thrust)

daemonize('vs_ctrl', main)
