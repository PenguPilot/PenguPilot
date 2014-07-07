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
  
 GVFA Writer Service

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """

import zmq
from scl import generate_map
from misc import daemonize, user_data_dir
from msgpack import loads
from threading import Thread


data = {}
writing = False


def write_file():
   global writing
   writing = True
   f = file('/root/.PenguPilot/config/gvfa_state.txt', 'w')
   print 'writing'
   for i in range(36):
      for j in range(72):
         for k in range(72):
            s = '%d %d %d %f %f %f\n' % (i, j, k, data[i, j, k, 0], data[i, j, k, 1], data[i, j, k, 2])
            f.write(s)
   f.close()
   print 'done'
   writing = False


def main(name):
   global data
   print 'initializing data'
   for i in range(36):
      for j in range(72):
         for k in range(72):
            data[i, j, k, 0] = 0
            data[i, j, k, 1] = 0
            data[i, j, k, 2] = 9.806650
   print 'loading data'
   for line in file('/root/.PenguPilot/config/gvfa_state.txt'):
      n, e, u = map(float, line.split(' ')[3:6])
      i, j, k = map(int, line.split(' ')[0:3])
      data[i, j, k, 0] = n
      data[i, j, k, 1] = e
      data[i, j, k, 2] = u
   
   print 'starting'
   socket = generate_map(name)['gvfa']
   c = 0
   while True:
      n, e, u, i, j, k = loads(socket.recv())
      data[i, j, k, 0] = n
      data[i, j, k, 1] = e
      data[i, j, k, 2] = u
      if (c % 5000) == 0 and not writing:
         t = Thread(target = write_file)
         t.start()
      c += 1

main('gvfa')
daemonize('gvfa', main)

