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
from time import sleep
from geomath import LinearInterpolation, gps_add_meters, gps_meters_offset
from numpy import array, zeros
from activity import Activity, StabMixIn

class MoveActivity(Activity, StabMixIn):


   def __init__(self, autopilot):
      Activity.__init__(self, autopilot)
      self.canceled = False

   def run(self):
      type, x, y = self.autopilot.arg
      ap = self.autopilot
      if type == 'spr': # starting point relative
         ap.start_pos = [ap.pse.data[4], ap.pse.data[6]]
         ap.api.set_hp([x - ap.start_pos[0], y - ap.start_pos[1]])
 
      #self.stabilize()
      
      if not self.canceled:
         ap.fsm.handle('done')

   def _cancel(self):
      self.canceled = True

