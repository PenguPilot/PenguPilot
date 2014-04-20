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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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


from sys import stdin
from numpy import array
import numpy as np
from scl import generate_map
from opcd_interface import OPCD_Interface


def read_array():
   line = stdin.readline()
   if not line:
      return
   arr = line.split(' ')
   arr[3] = arr[3][0:-1]
   return arr


header = read_array()
if not header:
   raise ValueError("could not read txt file header")
indices = []
for name in ['current', 'mag_cal_x', 'mag_cal_y', 'mag_cal_z']:
   indices.append(header.index(name))

opcd = OPCD_Interface(generate_map('opcd_shell')['ctrl'])

 
data = np.loadtxt(stdin, usecols = indices)

current = np.asarray(data[:, 0])
mag_x = np.asarray(data[:, 1])
mag_y = np.asarray(data[:, 2])
mag_z = np.asarray(data[:, 3])

A = np.vstack([current, np.ones(len(current))]).T
a1, b1 = np.linalg.lstsq(A, mag_x)[0]
a2, b2 = np.linalg.lstsq(A, mag_y)[0]
a3, b3 = np.linalg.lstsq(A, mag_z)[0]

opcd.set('pilot.cmc.scale_x', float(a1))
opcd.set('pilot.cmc.scale_y', float(a2))
opcd.set('pilot.cmc.scale_z', float(a3))
opcd.set('pilot.cmc.bias', float(min(current)))
opcd.persist()
