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
  
 Console Logger

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import sys
from scl import scl_get_socket
from os.path import basename
from msgpack import loads


def logdata_2_string(log_data):
   LOG_LEVEL_NAMES = ["ERROR", " WARN", " INFO", "DEBUG"];
   level_name = LOG_LEVEL_NAMES[log_data[1]]
   file = basename(log_data[2])
   return "[%s] %s|%s,%d: %s" % (level_name, log_data[0], file, log_data[3], log_data[4])



if __name__ == '__main__':
   socket = scl_get_socket('log_data_pub', 'sub')
   while True:
      try:
         log_data = loads(socket.recv())
         print logdata_2_string(log_data)
      except KeyboardInterrupt:
         break

