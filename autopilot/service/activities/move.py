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
 
 Movement Activity Class

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from math import hypot
from time import sleep, time
from geomath import LinearInterpolation, gps_add_meters, gps_meters_offset
from numpy import array, zeros
from activity import Activity, StabMixIn
from pylogger import *

class MoveActivity(Activity, StabMixIn):


   def __init__(self, autopilot):
      Activity.__init__(self, autopilot)
      self.canceled = False

   def run(self):
      type, x, y = self.autopilot.arg
      ap = self.autopilot
      timer_timeout = 20
      
      hp_err = SCL_Reader('hp_ctrl_err', 'sub')
      sleep(0.1)
      
      if type == 'cpr': # current point relative
         current_pos = [ap.pse.data[4], ap.pse.data[6]]
         ap.api.set_hp([current_pos[0] + x, current_pos[1] + y])
      if type == 'spr': #starting point relative
        move_pos = [x + ap.start_pos[0], y + ap.start_pos[1]]
        ap.api.set_hp(move_pos)

      sleep(0.1)
      
      start_time = time()
      timer = 0
      while ( abs(err.data[0]) > 0.15 or abs(err.data[1]) > 0.15) and timer < timer_timeout:
        timer = time - start_time
      if timer > timer_timeout:
        log(LL_INFO, "move timeout")
      ap.api.set_hp([ap.pse.data[4], ap.pse.data[6])
      #self.stabilize()
      
      if not self.canceled:
         ap.fsm.handle('done')

   def _cancel(self):
      self.canceled = True

