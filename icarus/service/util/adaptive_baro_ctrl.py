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
 
 Adaptive Barometric Control

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


class AdaptiveBaroCtrl:


   def __init__(self, baro_setpoint, ground_min, kp):
      self.baro_setpoint = baro_setpoint
      self.baro_shift = 0.0
      self.ground_min = ground_min
      self.kp = kp


   def calc(self, ground_dist):
      error = ground_dist - self.ground_min
      self.baro_shift = self.baro_shift - self.kp * error
      if self.baro_shift < 0:
         self.baro_shift = 0
      return self.baro_setpoint + self.baro_shift

