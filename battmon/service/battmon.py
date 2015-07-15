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
  
 Battery Monitor

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from misc import Hysteresis
from scl import scl_get_socket, SCL_Reader
from opcd_interface import OPCD_Interface
from misc import daemonize
from os import system
from time import time
from scheduler import sched_set_prio
from pp_prio import PP_PRIO_6


warning_sent = False


def warning():
   global warning_sent
   if not warning_sent:
      warning_sent = True   
      msg = 'WARNING: SYSTEM BATTERY VOLTAGE IS LOW; IMMEDIATE SHUTDOWN REQUIRED'
      system('echo "%s" | wall' % msg)


def main(name):
   sched_set_prio(PP_PRIO_6)
   current_reader = SCL_Reader('current', 'sub', [0.0])
   voltage_socket = scl_get_socket('voltage', 'sub')
   battery_socket = scl_get_socket('battery', 'pub')
   opcd = OPCD_Interface()
   hysteresis = Hysteresis(opcd.get('battmon.low_voltage_hysteresis'))
   cells = opcd.get('battmon.cells')
   low_cell_voltage_idle = opcd.get('battmon.low_cell_voltage_idle')
   low_cell_voltage_load = opcd.get('battmon.low_cell_voltage_load')
   current_treshold = opcd.get('battmon.current_treshold')
   vmax = cells * 4.1
   voltage = voltage_socket.recv()[0]
   vmin_idle = cells * low_cell_voltage_idle
   vmin_load = cells * low_cell_voltage_load
   time_prev = time()
   critical = False
   while True:
      current = current_reader.data[0]
      voltage = voltage_socket.recv()[0]
      time_prev = time()
      if current < current_treshold:
          vmin = vmin_idle
      else:
          vmin = vmin_load
      if voltage < vmin:
         critical = hysteresis.set()
      else:
         hysteresis.reset()
      if critical:
         warning()
      percent = min(1.0, max(0.0, (voltage - vmin) / (vmax - vmin)))
      battery_socket.send([percent, critical])


daemonize('battmon', main)

