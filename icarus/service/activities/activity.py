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
 
 Activity Class

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from time import sleep
from threading import Thread, current_thread
from core_pb2 import *
from math import sqrt, hypot


class Activity(Thread):

   def __init__(self, icarus):
      Thread.__init__(self)
      self.icarus = icarus
      self.daemon = True

   def _cancel(self):
      pass

   def cancel_and_join(self):
      if self != current_thread():
         self._cancel()
         self.join()


class StabMixIn:
   
   LAT_STAB_EPSILON = 3.0
   ALT_STAB_EPSILON = 10.4
   YAW_STAB_EPSILON = 100.3 # TODO: revert
   POLLING_TIMEOUT = 0.1
   STAB_COUNT = 20

   def stabilize(self):
      core = self.icarus.core
      mon_data = self.icarus.mon_data
      count = 0
      while True:
         sleep(self.POLLING_TIMEOUT)
         count += 1
         if count == self.STAB_COUNT:
            break
         if self.canceled:
            return
         # read error values from core:
         x_err, y_err = mon_data.x_err, mon_data.y_err
         alt_err = mon_data.z_err
         yaw_err = mon_data.yaw_err
         # reset counter if one of the errors becomes too huge:
         if abs(alt_err) > self.ALT_STAB_EPSILON:
            print 'alt instable', alt_err, count
            count = 0
         elif hypot(x_err, y_err) > self.LAT_STAB_EPSILON:
            print 'gps instable', x_err, y_err, count
            count = 0
         elif abs(yaw_err) > self.YAW_STAB_EPSILON:
            print 'yaw instable', yaw_err, count
            count = 0
      print 'stabilized'

