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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from msgpack import dumps
from scl import generate_map
from opcd_interface import OPCD_Interface
from misc import daemonize
from time import sleep


class TWL4030_MADC:

   def __init__(self, adc_id):
      self.path = '/sys/class/hwmon/hwmon0/device/in%d_input' % adc_id

   def read(self):
      file = open(self.path)
      try:
         val = int(file.read())
      except:
         val = 17.0
      file.close()
      return val


def main(name):
   map = generate_map(name)
   socket = map['power']
   opcd = OPCD_Interface(map['opcd_ctrl'], 'overo_quad')
   voltage_adc = TWL4030_MADC(opcd.get('voltage_channel'))
   current_adc = TWL4030_MADC(opcd.get('current_channel'))
   voltage_lambda = eval(opcd.get('adc_to_voltage'))
   current_lambda = eval(opcd.get('adc_to_current'))
   while True:
      try:
         sleep(0.2)
         voltage = voltage_lambda(voltage_adc.read())  
         current = current_lambda(current_adc.read())
         state = [voltage,  # 0 [V]
                  current]  # 1 [A]
         socket.send(dumps(state))
      except:
         pass

daemonize('twl4030_madc', main)

