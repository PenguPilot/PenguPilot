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
  
 Reads latest MessagePack Logfile, computes MAG/ACC 
 Calibration and commits calibration to OPCD, if valid

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


print 'loading acc/mag data'
from msgpack import Unpacker
from misc import msgpack_lastlog_path as log_path
unpacker = Unpacker(open(log_path))
header = unpacker.next()

mag_indices = []
for name in ['mag_x', 'mag_y', 'mag_z']:
   mag_indices.append(header.index(name))
acc_indices = []
for name in ['acc_x', 'acc_y', 'acc_z']:
   acc_indices.append(header.index(name))

from numpy import array
mag_list = []
acc_list = []
for msg in unpacker:
   mag = []
   for i in mag_indices:
      mag.append(msg[i])
   mag_list.append(array(mag))
   acc = []
   for i in acc_indices:
      acc.append(msg[i])
   acc_list.append(array(acc))


print 'computing calibration'
from cal_lib import Calibration
mag_cal = Calibration(mag_list)
acc_cal = Calibration(acc_list)

print 'checking calibration'
from math import isnan
from sys import exit
mag_cal_data = mag_cal.get_cal()
for val in mag_cal_data:
   if isnan(val):
      print 'mag calibration invalid'
      exit(1)
acc_cal_data = acc_cal.get_cal()
for val in acc_cal_data:
   if isnan(val):
      print 'acc calibration invalid'
      exit(2)

print 'saving calibration via OPCD'
from scl import generate_map
from opcd_interface import OPCD_Interface
opcd = OPCD_Interface(generate_map('opcd_shell')['ctrl'])

prefix = 'pilot.cal.'
i = 0
for sensor in ['mag', 'acc']:
   for type in ['bias', 'scale']:
      for axis in 'xyz':
         opcd.set(prefix + sensor + '_' + type + '_' + axis, cal_data[i])
         i += 1
opcd.persist()

