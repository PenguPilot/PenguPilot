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

 reads mag/acc log text file from stdin and writes
 calibration to stdout

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 Development of this code has been supported by the Department of Computer Science,
 Universita' degli Studi di Torino, Italy within the Piemonte Project
 http://www.piemonte.di.unito.it

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """

from scl import generate_map
from numpy import array
from msgpack import loads
from cal_math import Calibration
from math import isnan
from sys import exit


socket = generate_map('marg_cal_tool')['marg_raw']
acc_list = []
mag_list = []
print 'collecting sensor data'
try:
   while True:
      msg = loads(socket.recv())
      acc_list.append(array(msg[4:7]))
      mag_list.append(array(msg[7:10]))
except:
   pass

# compute calibration:
acc_cal = Calibration(acc_list)
mag_cal = Calibration(mag_list)

# verify calibration:
acc_cal_data = acc_cal.get_cal()
for val in acc_cal_data:
   if isnan(val):
      print 'calibration invalid'
      exit(2)
mag_cal_data = mag_cal.get_cal()
for val in mag_cal_data:
   if isnan(val):
      print 'calibration invalid'
      exit(1)

# print calibration to stdout:
cal_data = list(acc_cal_data) + list(mag_cal_data)
i = 0
for sensor in ['acc', 'mag']:
   for type in ['bias', 'scale']:
      for axis in 'xyz':
         print sensor + '_' + type + '_' + axis, cal_data[i]
         i += 1

