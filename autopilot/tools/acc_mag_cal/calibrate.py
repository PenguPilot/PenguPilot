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

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

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


def read_array():
   line = stdin.readline()
   if not line:
      return
   arr = line.split(' ')
   arr[5] = arr[5][0:-1]
   return arr


# read acc/mag data to array:
from sys import stdin
header = read_array()
if not header:
   raise ValueError("could not read txt file header")
mag_indices = []
for name in ['mag_x', 'mag_y', 'mag_z']:
   mag_indices.append(header.index(name))
acc_indices = []
for name in ['acc_x', 'acc_y', 'acc_z']:
   acc_indices.append(header.index(name))

from numpy import array
mag_list = []
acc_list = []
try:
   while True:
      msg = map(float, read_array())
      mag = []
      for i in mag_indices:
         mag.append(msg[i])
      mag_list.append(array(mag))
      acc = []
      for i in acc_indices:
         acc.append(msg[i])
      acc_list.append(array(acc))
except:
   pass

# compute calibration:
from cal_math import Calibration
mag_cal = Calibration(mag_list)
acc_cal = Calibration(acc_list)

# verify calibration:
from math import isnan
from sys import exit
mag_cal_data = mag_cal.get_cal()
for val in mag_cal_data:
   if isnan(val):
      print 'calibration invalid'
      exit(1)
acc_cal_data = acc_cal.get_cal()
for val in acc_cal_data:
   if isnan(val):
      print 'calibration invalid'
      exit(2)

# print calibration to stdout:
cal_data = list(mag_cal_data) + list(acc_cal_data)
i = 0
for sensor in ['mag', 'acc']:
   for type in ['bias', 'scale']:
      for axis in 'xyz':
         print sensor + '_' + type + '_' + axis, cal_data[i]
         i += 1


