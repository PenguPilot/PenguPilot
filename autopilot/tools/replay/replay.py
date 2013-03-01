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

 Functions for running Replays

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from os import environ, system
from misc import user_data_dir
pre = environ['PENGUPILOT_PATH'] + '/'


def replay(f):
   system(pre + 'svctrl/svctrl.py --start opcd')
   system(pre + 'svctrl/svctrl.py --start debug_logger')
   system(pre + 'autopilot/services/autopilot %s' % argv[1])
   system(pre + 'svctrl/svctrl.py --stop debug_logger')
   

if __name__ == '__main__':
   from sys import argv
   assert len(argv) == 2
   replay(argv[1])
   system(pre + 'autopilot/tools/msgpack_to_txt.py < %s | ' % (user_data_dir + '/log/autopilot_debug.msgpack') + pre + 'autopilot/tools/txt_col.py raw_n pos_n > kalman.txt')

