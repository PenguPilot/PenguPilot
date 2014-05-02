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
  
 Blackbox Service

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from os import sep, symlink, unlink, fsync
from scl import generate_map
from misc import daemonize, user_data_dir
from datetime import datetime
from sys import argv

def main(name):
   socket = generate_map(name)['blackbox']
   prefix = user_data_dir + sep + 'log' + sep
   try:
      now = datetime.today().isoformat().replace(':', '')
      symlink_file = prefix + 'blackbox_last.msgpack'
      try:
         unlink(symlink_file)
      except:
         pass
      if len(argv) > 1:
         new_file = prefix + 'blackbox_%s_%s.msgpack' % (now, argv[1])
      else:
         new_file = prefix + 'blackbox_%s.msgpack' % now
      symlink(new_file, symlink_file)
      f = open(new_file, "wb")
      while True:
         f.write(socket.recv())
         #f.flush()
         #fsync(f.fileno())
   finally:
      try:
         f.close()
      except:
         pass

daemonize('blackbox', main)

