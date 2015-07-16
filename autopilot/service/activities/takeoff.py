"""
  ___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autoap |
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


class TakeoffActivity(Activity, StabMixIn):


   def __init__(self, fsm, autoap):
      Activity.__init__(self, autoap)
      self.canceled = False
      self.fsm = fsm


   def _cancel(self):
      self.canceled = True


   def run(self):
      api = self.autopilot.api
      arg = self.autopilot.arg
      if arg:
         vp_max = 4.0
         vp_target = min(vp_max, arg)
      else:
         vp_target = 1.0

      # start motors and wait for
      api.mot_en(True)
      while ap.motors_state.recv() != 2:
         if self.canceled:
            api.mot_en(False)
            return

      # "point of no return":
      ap.home_pos = [ap.pse.data[4], ap.pse.data[6]]
      api.set_hp(ap.home_pos)
      api.set_ys(0.0)

      # increase altitude setpoint:
      vp = -1.0
      while vp < vp_target:
         api.set_vp(vp)
         vp += 0.05
         sleep(0.1)
      api.set_vp(vp_target)
      #self.stabilize()
      self.fsm.handle('done')

