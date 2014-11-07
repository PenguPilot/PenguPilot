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
  
 TWL4030 Power Publisher Service
 - monitors system power
 - estimates battery state of charge (SOC)
 - predicts remaining battery lifetime
 - manages main power and lights

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from misc import *
from msgpack import loads, dumps
from scl import generate_map
from opcd_interface import OPCD_Interface
from misc import daemonize
from os import system


warning_sent = False

def warning():
   global warning_sent
   if not warning_sent:
      warning_sent = True   
      msg = 'WARNING: SYSTEM BATTERY VOLTAGE IS LOW; IMMEDIATE SHUTDOWN REQUIRED'
      system('echo "%s" | wall' % msg)


def main(name):
   map = generate_map(name)
   power_socket = map['power']
   powerman_socket = map['powerman']
   opcd = OPCD_Interface(map['opcd_ctrl'])
   hysteresis = Hysteresis(opcd.get('powerman.low_voltage_hysteresis'))
   cells = opcd.get('powerman.battery_cells')
   low_cell_voltage_idle = opcd.get('powerman.battery_low_cell_voltage_idle')
   low_cell_voltage_load = opcd.get('powerman.battery_low_cell_voltage_load')
   battery_current_treshold = opcd.get('powerman.battery_current_treshold')
   capacity = opcd.get('powerman.battery_capacity')
   vmin = 13.2
   vmax = 16.4
   voltage, current = loads(power_socket.recv())
   batt = min(1.0, max(0.0, (voltage - vmin) / (vmax - vmin)))
   consumed = (1.0 - batt) * capacity
   low_battery_voltage_idle = cells * low_cell_voltage_idle
   low_battery_voltage_load = cells * low_cell_voltage_load
   time_prev = time()
   critical = False
   while True:
      voltage, current = loads(power_socket.recv())
      consumed += current / 3600.0 * (time() - time_prev)
      time_prev = time()
      if voltage < low_battery_voltage_idle and current < battery_current_treshold or voltage < low_battery_voltage_load and current >= battery_current_treshold:
         critical = hysteresis.set()
      else:
         hysteresis.reset()
      remaining = capacity - consumed
      if remaining < 0:
         remaining = 0
      if critical:
         warning()
      state = [voltage,   # 0 [V]
               current,   # 1 [A]
               remaining, # 2 [Ah]
               critical]  # 3 [Bool]
      powerman_socket.send(dumps(state))


daemonize('powerman', main)

