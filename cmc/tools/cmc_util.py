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

 reads mag/current log text file from stdin and writes
 calibration to opcd

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from numpy import array
import numpy as np
from scl import scl_get_socket
from opcd_interface import OPCD_Interface
from time import sleep
from threading import Thread
from msgpack import loads


mag = [0.0, 0.0, 0.0]
def mag_reader():
   global mag
   socket = scl_get_socket('mag_adc_cal', 'sub')
   while True:
      mag = loads(socket.recv())

current = 0.0
def curr_reader():
   global current
   socket = scl_get_socket('current', 'sub')
   while True:
      current = loads(socket.recv())[0]

tc = Thread(target = curr_reader)
tc.daemon = True
tc.start()

tm = Thread(target = mag_reader)
tm.daemon = True
tm.start()

print 'waiting for current/mag data'
while True:
   if current != 0.0 and mag[0] != 0.0 and mag[1] != 0.0 and mag[2] != 0.0:
       break
print 'starting calibration sweep'
exit(1)



opcd = OPCD_Interface(generate_map('opcd_shell')['opcd_ctrl'])

current = np.asarray(data[:, 0])
mag_x = np.asarray(data[:, 1])
mag_y = np.asarray(data[:, 2])
mag_z = np.asarray(data[:, 3])

A = np.vstack([current, np.ones(len(current))]).T
a1, b1 = np.linalg.lstsq(A, mag_x)[0]
a2, b2 = np.linalg.lstsq(A, mag_y)[0]
a3, b3 = np.linalg.lstsq(A, mag_z)[0]

opcd.set('autopilot.cmc.scale_x', float(a1))
opcd.set('autopilot.cmc.scale_y', float(a2))
opcd.set('autopilot.cmc.scale_z', float(a3))
opcd.set('autopilot.cmc.bias', float(min(current)))
opcd.persist()
