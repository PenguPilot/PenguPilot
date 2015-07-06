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

 Drives the Motors to perform Current Magnetometer Calibration (CMC)
 WARNING: this script starts the motors.
 You need to fix the MAV on the ground!

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from geomath import rad2deg
from numpy import array
import numpy as np
from scl import scl_get_socket, SCL_Reader
from opcd_interface import OPCD_Interface
from time import sleep
from threading import Thread
from msgpack import loads, dumps


thrust_socket = scl_get_socket('thrust', 'pub')
torques_socket = scl_get_socket('torques', 'pub')
mot_en_socket = scl_get_socket('mot_en', 'pub')
mag = SCL_Reader('mag_adc_cal', 'sub', [0.0, 0.0, 0.0])
curr = SCL_Reader('current', 'sub', [0.0])
int_en = SCL_Reader('int_en', 'sub', 1)
orientation = SCL_Reader('orientation', 'sub', [0.0, 0.0, 0.0])

while True:
   if curr.data[0] != 0.0 and mag.data[0] != 0.0 and mag.data[1] != 0.0 and mag.data[2] != 0.0:
       break
print 'starting calibration sweep'

# start motors:
sleep(1)
mot_en_socket.send(dumps(1))
for i in range(10000):
   thrust_socket.send(dumps(0.0))
   torques_socket.send(dumps([0.0, 0.0, 0.0]))

# increase gas and store measurements:
o_start = orientation.data[0]
meas = []
try:
   thrust = 0.0
   while True:
      thrust += 0.01
      thrust_socket.send(dumps(thrust))
      torques_socket.send(dumps([0.0, 0.0, 0.0]))
      meas.append([curr.data[0]] + mag.data)
      sleep(0.01)
      print 'error:', rad2deg(o_start - orientation.data[0])
      #if not int_en.data:
      #    break
finally:
   mot_en_socket.send(dumps(0))

opcd = OPCD_Interface()

meas = array(meas)
current = np.asarray(meas[:, 0])
mag_x = np.asarray(meas[:, 1])
mag_y = np.asarray(meas[:, 2])
mag_z = np.asarray(meas[:, 3])

A = np.vstack([current, np.ones(len(current))]).T
a1, b1 = np.linalg.lstsq(A, mag_x)[0]
a2, b2 = np.linalg.lstsq(A, mag_y)[0]
a3, b3 = np.linalg.lstsq(A, mag_z)[0]

opcd.set('cmc.scale_x', float(a1))
opcd.set('cmc.scale_y', float(a2))
opcd.set('cmc.scale_z', float(a3))
opcd.set('cmc.bias', float(min(current)))
opcd.persist()
