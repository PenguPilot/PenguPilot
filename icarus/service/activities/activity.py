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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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
from pilot_pb2 import *
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
 
   STAB_EPSILON = 0.25
   POLLING_TIMEOUT = 0.1
   STAB_COUNT = 20

   def stabilize(self):
      pilot = self.icarus.pilot
      count = 0
      while True:
         sleep(self.POLLING_TIMEOUT)
         count += 1
         if count == self.STAB_COUNT:
            break
         if self.canceled:
            print 'canceled'
            return
         # read error values from pilot:
         n_err, e_err, u_err = pilot.mon[5], pilot.mon[6], pilot.mon[7]
         print 'X:', n_err, e_err, u_err
         # reset counter if one of the errors becomes too huge:
         if abs(u_err) > self.STAB_EPSILON:
            print 'u instable', u_err, count
            count = 0
         if hypot(n_err, e_err) > self.STAB_EPSILON:
            print 'n/e instable', n_err, e_err, count
            count = 0
      print 'stabilized'

