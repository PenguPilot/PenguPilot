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
  
 MAG Calibration Utility

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
from cal_math import Calibration
from opcd_interface import OPCD_Interface
from math import isnan

opcd = OPCD_Interface(scl_get_socket('opcd_ctrl', 'req'))
points = []

print 'collecting magnetometer data .. press ctrl+c when finished'
socket = scl_get_socket('mag_raw', 'sub')
while True:
   try:
      vec = loads(socket.recv())
      points.append(vec)
   except:
      break

c = Calibration(points)
cal = c.get_cal()
names = ['mag_bias_x', 'mag_bias_y', 'mag_bias_z', 'mag_scale_x', 'mag_scale_y', 'mag_scale_z']

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
      opcd.set('mag_adc_cal.' + names[i], val)
   print 'magnetometer calibration complete, please type persist() in pp_opcd_shell to save'

