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
  
 Pilot Shell Setup Script

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import atexit
import os
import readline
import rlcompleter

from scl import generate_map
from pilot_pb2 import *
from pilot_interface import PilotInterface
from misc import user_data_dir


# set-up command history:
_path = user_data_dir + os.sep + 'pilot_shell.history'
_history = os.path.expanduser(_path)
def _save_history(historyPath = _history):
   readline.write_history_file(_history)
if os.path.exists(_history):
   readline.read_history_file(_history)
readline.parse_and_bind("tab: complete")
atexit.register(_save_history)

def monitor():
   mon_data = MonData()
   try:
      while True:
         i.mon_read(mon_data)
         print mon_data
   except:
      pass

_map = generate_map('pilot_shell')
_ctrl_socket = _map['ctrl']
_mon_socket = _map['mon']
i = PilotInterface(_ctrl_socket, _mon_socket)


print 'type help(i) for help'
