#!/usr/bin/env python
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
  
 ACC G Magnitude Verification Utility

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from scl import scl_get_socket
from msgpack import loads

socket = scl_get_socket('acc_cal', 'sub')
while True:
   try:
      vec = loads(socket.recv())
      points.append(vec)
      print len(points)
   except:
      break

c = Calibration(points)
cal = c.get_cal()
names = ['acc_bias_x', 'acc_bias_y', 'acc_bias_z', 'acc_scale_x', 'acc_scale_y', 'acc_scale_z']

# detect calibration failure:
fail = False
for i in range(len(names)):
   val = float(cal[i])
   if isnan(val):
      print 'bad calibration'
      fail = True
      break

# write opcd values:
if not fail:
   for i in range(len(names)):
      val = float(cal[i])
      opcd.set('acc_adc_cal.' + names[i], val)
   print 'accelerometer calibration complete, please type persist() in pp_opcd_shell to save'

