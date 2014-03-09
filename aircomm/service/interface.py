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
  
 Ed_nrf Wrapper Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from ed_nrf.lib.cdc import CDC_nRF
from time import sleep


class Interface:

   def __init__(self, dev_path):
      self.nrf = CDC_nRF(dev_path)
      self.nrf.writeRegister(1, 0)
      self.nrf.setPower(True) # enable power
      self.nrf._bus.setDTR(True) # enable transparent mode
      self.nrf._bus.timeout = 0.01 # disable read/write timeouts

   def send(self, data):
      self.nrf._bus.write(data)

   def receive(self):
      return self.nrf._bus.read(1024)

