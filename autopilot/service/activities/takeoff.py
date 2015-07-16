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
 
 Takeoff Activity Class

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from activity import Activity, StabMixIn
from time import sleep
from logging import debug as log_debug, info as log_info, warning as log_warn, error as log_err
from ctrl_api import *


class TakeoffActivity(Activity, StabMixIn):


   def __init__(self, fsm, autopilot):
      Activity.__init__(self, autopilot)
      self.canceled = False
      self.fsm = fsm


   def _cancel(self):
      self.canceled = True


   def run(self):
      pilot = self.autopilot
      arg = pilot.arg
      if arg:
         u_max = 3.5
         if arg > u_max:
            u_setpoint = u_max
         else:
            u_setpoint = arg
      else:
         u_setpoint = 1.0

      mot_en(True)

      if self.canceled:
         pilot.stop_motors()
         log_error('take-off canceled');
         return

      # "point of no return":
      pilot.start_pos = [pilot.pse.data[4], pilot.pse.data[6]]
      set_hp(pilot.start_pos)
      set_ys(0.0)

      # set new altitude setpoint and stabilize:
      u_setp = -1.0
      while u_setp < u_setpoint:
         set_vp(u_setp)
         u_setp += 0.05
         sleep(0.1)
      pilot.set_ctrl_param(u_setpoint)
      #self.stabilize()
      self.fsm.handle('done')

