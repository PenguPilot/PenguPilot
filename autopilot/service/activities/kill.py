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
 
 Kill Activity Class

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


class KillActivity(Activity, StabMixIn):


   def __init__(self, autopilot):
      Activity.__init__(self, autopilot)
      self.canceled = False

   def _cancel(self):
      self.canceled = True

   def run(self):
      ap = self.autopilot
      api = self.autopilot.api
      fsm = self.autopilot.fsm
      #take care of all controlles - use safe values (in standing mode)
      current_pos = [ap.pse.data[4], ap.pse.data[6]]
      api.set_hp(current_pos)
      api.set_ys(0.0)
      #api.set_hs(0.0)
      api.set_vp(-1.0)
      # stop motors
      api.mot_en(False)
      fsm.handle('done')
