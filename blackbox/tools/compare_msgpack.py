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
  
 MessagPack Log File Compare Utility

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sys import argv
from msgpack import Unpacker


def unpack_gen(file, size):
   u = Unpacker()
   while True:
      data = file.read(size)
      if not data:
         break
      u.feed(data)
      for o in u:
         yield o


def unpack_file(f):
   data = []
   for o in unpack_gen(f, 1024):
      data.append(o)
   return data


assert len(argv) == 3

a = unpack_file(open(argv[1]))
b = unpack_file(open(argv[2]))

ha = a[0]
hb = b[0]

a = a[1:]
b = b[1:]

print 'lengths', len(a), len(b)

for i in range(len(a) - 1):
   for j in range(len(ha)):
      try:
         if a[i][j] != b[i][j]:
            print i, ha[j], a[i][j], b[i][j]
      except IndexError:
         pass

