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
  
 Generic optimizer with configurable mutation rate

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from random import uniform
from scl import scl_get_socket
from opcd_interface import OPCD_Interface
import sys
from copy import copy

def optimize(rate, samples, prefix, names, err_socket, mse_indices):
   
   opcd = OPCD_Interface()
   rs_ctrl_err = scl_get_socket(err_socket, 'sub')
   vec_best = [ opcd.get(prefix + n) for n in names]
   vec = vec_best
   mse_min = sys.float_info.max
   best_log = []
   try:
      while True:
         vec = map(lambda x: x * uniform(1.0 - rate, 1.0 + rate), vec)
         # send new params to opcd:
         for i, n in zip(range(len(names)), names):
            opcd.set(prefix + n, vec[i])
         # read data and compute fitness:
         mse = 0.0
         for _ in range(samples):
            array = rs_ctrl_err.recv()
            for i in mse_indices:
               mse += array[i] ** 2
         mse /= samples
         print mse
         # evaluate mse:
         if mse < mse_min:
            best_log.append(mse)
            mse_min = mse
            vec_best = copy(vec)
            print vec_best
         else:
            # we did not improve;
            # use best vector as search starting point
            vec = copy(vec_best)
   except:
      for i, n in zip(range(len(names)), names):
         opcd.set(prefix + n, vec_best[i])
      #opcd.persist()
   return best_log
