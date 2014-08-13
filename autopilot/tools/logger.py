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
  
 Autopilot Console Logger

 Copyright (C) 2014 Tobias Simon

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import log_data_pb2
import sys
from scl import generate_map
from os.path import basename


def logdata_2_string(log_data):
   LOG_LEVEL_NAMES = ["ERROR", " WARN", " INFO", "DEBUG"];
   level_name = LOG_LEVEL_NAMES[log_data.level]
   file = basename(log_data.file)
   if log_data.details == 1:
      return "[%s] %s: %s" % (level_name, file, log_data.msg)
   elif log_data.details == 2:
      return "[%s] %s,%d: %s" % (level_name, file, log_data.line, log_data.msg)
   else:
      return "[%s] %s" % (level_name, log_data.msg)



if __name__ == '__main__':
   socket = generate_map('console_logger')['log_data_out']
   while True:
      try:
         log_data = log_data_pb2.log_data()
         raw_data = socket.recv()
         log_data.ParseFromString(raw_data)
         print logdata_2_string(log_data)
      except KeyboardInterrupt:
         break
      except Exception, e:
         print str(e)

