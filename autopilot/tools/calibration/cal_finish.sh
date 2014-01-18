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
# computes calibration from blackbox log file
# and saves computed values via OPCD
#
# Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology
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


script=`realpath "$0"`
dir=`dirname $script`
cd $dir

pp_svctrl --stop autopilot
pp_svctrl --stop blackbox

pp_msgpack_to_txt < ~/.PenguPilot/log/blackbox_last.msgpack | \
pp_filter_txt_col acc_x acc_y acc_z mag_x mag_y mag_z | \
./calibrate.py | \
./cal_to_opcd.py

