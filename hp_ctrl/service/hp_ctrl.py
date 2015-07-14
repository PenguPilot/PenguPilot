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
  
 Horizontal Position Control (using GPS)

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
from msgpack import dumps, loads
from opcd_interface import OPCD_Subscriber
from misc import daemonize
from pp_prio import PP_PRIO_5
from scheduler import sched_set_prio
from math import sin, cos
from pos_speed_est_neu import N_POS, E_POS


def main(name):
   sched_set_prio(PP_PRIO_5)
   opcd = OPCD_Subscriber()
   sp_n = SCL_Reader(name + '_sp_n', 'sub', 0.0)
   sp_e = SCL_Reader(name + '_sp_e', 'sub', 0.0)
   n_oe = SCL_Reader(name + '_n_oe', 'pull', 1)
   e_oe = SCL_Reader(name + '_e_oe', 'pull', 1)
   sp = [sp_n, sp_e]
   err = scl_get_socket(name + '_err', 'pub')
   hs_spp_n = scl_get_socket('hs_ctrl_spp_n', 'push')
   hs_spp_e = scl_get_socket('hs_ctrl_spp_e', 'push')
   ctrls = [PID_Ctrl(), PID_Ctrl()]
   pos_speed_est = scl_get_socket('pos_speed_est_neu', 'sub')
   while True:
      est = loads(pos_speed_est.recv())
      ne_pos = [est[N_POS], est[E_POS]]
      setpoints = [sp_n.data, sp_e.data]
      ne_ctrl = [0.0, 0.0]
      try:
         for c in range(2):
            ctrls[c].p = opcd[name + '.p']
            ne_ctrl[c] = ctrls[c].control(ne_pos[c], setpoints[c])
         err.send(dumps([ctrls[0].err, ctrls[1].err]))
      except:
         pass
      if n_oe.data:
         hs_spp_n.send(dumps(ne_ctrl[0]))
      if e_oe.data:
         hs_spp_e.send(dumps(ne_ctrl[1]))


daemonize('hp_ctrl', main)
