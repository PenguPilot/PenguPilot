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
 
 Landing Activity Class

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from time import sleep
from pilot_pb2 import *

from activity import Activity


class LandActivity(Activity):

   MIN_HOVERING_ALT = 0.57

   def __init__(self, icarus):
      Activity.__init__(self, icarus)

   def run(self):
      pilot = self.icarus.pilot
      mon_data = self.icarus.mon_data
      fsm = self.icarus.fsm
      
      pilot.set_ctrl_param(POS_Z_GROUND, self.MIN_HOVERING_ALT / 3.0)
      while mon_data.z_ground > self.MIN_HOVERING_ALT:
         sleep(self.POLLING_TIMEOUT)
      pilot.spin_down()
      fsm.handle('done')

