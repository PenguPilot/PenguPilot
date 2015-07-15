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
# This Script install Binaries to /usr/local/bin
#
# Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
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


cd /usr/local/bin
sudo ln -f -s $PENGUPILOT_PATH/opcd/tools/opcd_shell.sh pp_opcd_shell
sudo ln -f -s $PENGUPILOT_PATH/scripts/clear_pidfiles.sh pp_clear_pidfiles
sudo ln -f -s $PENGUPILOT_PATH/svctrl/svctrl.py pp_svctrl
sudo ln -f -s $PENGUPILOT_PATH/acc_cal/tools/acc_cal.py pp_acc_cal
sudo ln -f -s $PENGUPILOT_PATH/acc_cal/tools/acc_magnitude.py pp_acc_magnitude
sudo ln -f -s $PENGUPILOT_PATH/mag_adc_cal/tools/mag_adc_cal.py pp_mag_adc_cal
sudo ln -f -s $PENGUPILOT_PATH/scl/tools/scl_dump.py pp_scl_dump
sudo ln -f -s $PENGUPILOT_PATH/scl/tools/scl_list.py pp_scl_list
sudo ln -f -s $PENGUPILOT_PATH/scl/tools/scl_show_stats.py pp_scl_show_stats
sudo ln -f -s $PENGUPILOT_PATH/rc_cal/tools/rc_cal.py pp_rc_cal
sudo ln -f -s $PENGUPILOT_PATH/rc_cal/tools/rc_cal_dump.py pp_rc_cal_dump

