#!/bin/bash
#  ___________________________________________________
# |  _____                       _____ _ _       _    |
# | |  __ \                     |  __ (_) |     | |   |
# | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
# | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
# | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
# | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
# |                   __/ |                           |
# |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
# |___________________________________________________|
#
# Replay PenguPilot log file (argument 1) and validates output (dump to argument 2)
#
# Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

$PENGUPILOT_PATH/svctrl/svctrl.py --start opcd
$PENGUPILOT_PATH/svctrl/svctrl.py --start debug_logger
$PENGUPILOT_PATH/autopilot/services/autopilot $1
$PENGUPILOT_PATH/svctrl/svctrl.py --stop debug_logger
$PENGUPILOT_PATH/autopilot/tools/compare_msgpack.py $1 $HOME/.PenguPilot/log/autopilot_debug.msgpack > $2

