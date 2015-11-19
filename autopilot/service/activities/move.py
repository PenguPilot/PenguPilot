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
      MOV_ACCURACY = 0.15
      MOV_TIMEOUT = 15

      err = SCL_Reader('hp_ctrl_err', 'sub')
      sleep(0.1)

      if type == 'cpr': # current point relative
        current_pos = [ap.pse.data[4], ap.pse.data[6]]
        ap.api.set_hp([current_pos[0]+x, current_pos[1]+y])
      if type == 'spr': #starting point relative
        move_pos = [x + ap.home_pos[0], y + ap.home_pos[1]]
        ap.api.set_hp(move_pos)
     
      if type == 'global':
        start_pos = [ap.pse.data[4], ap.pse.data[6]]
        dest_pos  = [x, y]
        offset = gps_meters_offset(start_pos, dest_pos)
        ap.api.set_hp([start_pos[0]+offset[0], start_pos[1]+offset[1]])

      sleep(0.05)
      start_time = time.time()
      timer = 0
      while (abs(err.data[0]) > MOV_ACCURACY or abs(err.data[1]) > MOV_ACCURACY) and timer < MOV_TIMEOUT:
        timer = time.time() - start_time
        sleep(0.01)
      if timer > MOV_TIMEOUT:
          log(LL_INFO, 'move timeout')

      #self.stabilize()
      
      if not self.canceled:
         ap.fsm.handle('done')

   def _cancel(self):
      self.canceled = True

