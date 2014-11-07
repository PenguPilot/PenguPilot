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
  
 OLED Display Showing Various Information

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from time import sleep, time
from threading import Thread
from scl import generate_map
from misc import daemonize, RateTimer
from msgpack import loads
from gps_msgpack import *
from serialport import serialport_t, serial_open, serial_write_str
import os


socket_map = None


def gps_reader():
   global gps
   socket = socket_map['gps']
   while True:
      raw = socket.recv()
      gps = loads(raw)


def pm_reader():
   s = socket_map['power']
   global voltage, current
   while True:
      raw = s.recv()
      voltage, current = loads(raw)


def main(name):
   global socket_map
   socket_map = generate_map(name)

   t2 = Thread(target = pm_reader)
   t2.daemon = True
   t2.start()

   t3 = Thread(target = gps_reader)
   t3.daemon = True
   t3.start()
   
   ser = serialport_t()
   CSTOPB = 0000100
   serial_open(ser, "/dev/ttyO0", 100000, os.O_WRONLY, CSTOPB)
   while True:
      try:
         serial_write_str(ser, "H %d %d\n" % (int(current * 100), int(voltage * 100)))
         if fix(gps) >= 2:
            serial_write_str(ser, "G2 %d %d %d\n" % (int(gps[LAT] * 10000000.0), int(gps[LON] * 10000000.0), int(gps[SPEED] * 10.0)))
         if fix(gps) == 3:
            serial_write_str(ser, "G3 %d %d\n" % (int(gps[ALT] * 1000), 0))
      except:
         pass
      sleep(0.1)

#main('teensy_taranis')
daemonize('teensy_taranis', main)

