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
  
 Power Monitoring Hardware Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """



class ADC:

   def __init__(self, adc_id):
      self.path = '/sys/class/hwmon/hwmon0/device/in%d_input' % adc_id

   def read(self):
      return int(open(self.path).read())


class GPIO_Bank:

   def __init__(self, bus, dev):
      self.bus = bus
      self.dev = int(dev)
      self.state = 1

   def set_gpio(self, id, state):
      id = int(id)
      if id < 0 or id > 7:
         raise ValueError('expected id to be in [0 ... 7]')
      state = bool(state)
      if state:
         self.state |= 1 << id;
      else:
         self.state &= ~(1 << id)
      self.write()

   def write(self):
      self.bus.write_byte_data(self.dev, 1, self.state)
