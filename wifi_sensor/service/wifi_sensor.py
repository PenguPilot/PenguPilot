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
  
 Wifi Networks Sensor Daemon

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from iwlistparse2 import parse_cells
from subprocess import Popen, PIPE
from scl import generate_map
from time import sleep
from misc import daemonize
from msgpack import dumps


def main(name):
   socket = generate_map(name)['networks']
   while True:
      try:
         pipe = Popen(['iwlist', 'wlan0', 'scan'], stdout = PIPE).stdout
         cells = parse_cells(pipe.readlines())
         for cell in cells:
            try:
               sig = int(cell['Signal'][0:-3])
            except:
               sig = int(cell['Signal'][0:-4])
            pair = [cell['Address'] + '_' + cell['Name'], sig]
            socket.send(dumps(pair))
         sleep(1.0)
      except:
         pass


daemonize('wifi_sensor', main)

