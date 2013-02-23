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

 Replays a recorded file (arg 1) and dumps result to file (arg 2)

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sys import argv
assert len(argv) == 3

from os import environ, system
pre = environ['PENGUPILOT_PATH'] + '/'
from misc import user_data_dir

system(pre + 'svctrl/svctrl.py --start opcd')
system(pre + 'svctrl/svctrl.py --start debug_logger')
system(pre + 'autopilot/services/autopilot %s' % argv[1])
system(pre + 'svctrl/svctrl.py --stop debug_logger')
system(pre + 'autopilot/tools/compare_msgpack.py %s ' % argv[1] 
    + user_data_dir() + '/log/autopilot_debug.msgpack > %s' % argv[2])

