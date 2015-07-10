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
  
 Vertical Position Control (using Ultrasonic, Barometer, Elevation Map)

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
from physics import G_CONSTANT
from misc import daemonize
from pp_prio import PP_PRIO_3
from scheduler import sched_set_prio
from pylogger import *
from pos_speed_est_neu import BARO_POS, ULTRA_POS


# ['ultra', 1.0] # relative to ground, ultrasonic sensor or laser/radar altimeter
# ['baro_rel', 10.0] # relative to initial baro value
# ['baro_abs', 590.0] # absolute position above MSL (baro)
# ['elev', 10.0] # relative to elevation map, 10m safety distance


def main(name):
   sched_set_prio(PP_PRIO_3)
   pid = PID_Ctrl()
   opcd = OPCD_Subscriber()
   elev = SCL_Reader('elev', 'sub', None)
   pos_mode_sp = SCL_Reader(name + '_sp', 'sub', ['ultra', -10.0])
   pos_oe = SCL_Reader(name + '_oe', 'pull', 1)
   vs_ctrl_spp = scl_get_socket('vs_ctrl_spp', 'push')
   err = scl_get_socket(name + '_err', 'pub')
   baro_pos_start = None
   elev_start = None
   pos_speed_est = scl_get_socket('pos_speed_est_neu', 'sub')
   while True:
      est = loads(pos_speed_est.recv())
      ultra_pos, baro_pos = est[ULTRA_POS], est[BARO_POS]
      if not baro_pos_start:
         baro_pos_start = baro_pos
      try:
         if not elev_start:
            elev_start = elev.data
      except:
         print 'x'
         continue
      pid.p = opcd[name + '.p']
      mode, sp = pos_mode_sp.data
      print mode
      if mode == 'ultra':
         ctrl = pid.control(ultra_pos, sp)
      elif mode == 'baro_rel':
         ctrl = pid.control(baro_pos - baro_pos_start, sp)
      elif mode == 'baro_abs':
         ctrl = pid.control(baro_pos, sp)
      else: # elev
         elev_rel = elev_start - elev.data
         baro_rel = baro_pos - baro_pos_start
         ctrl = pid.control(baro_rel, elev_rel + sp)
      err.send(dumps(pid.err))
      if pos_oe.data:
         vs_ctrl_spp.send(dumps(ctrl))


daemonize('vp_ctrl', main)
