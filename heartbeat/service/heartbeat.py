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
  
 Heartbeat Emitter

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from time import sleep, time
from psutil import cpu_percent
from threading import Thread, Lock
from scl import generate_map, RateTimer
from math import sin, cos, pi
from misc import daemonize
from msgpack import loads, dumps
from aircomm_shared import BCAST_NOFW, HEARTBEAT
from gps_msgpack import *

socket_map = None
voltage = 17.0
current = 0.4
critical = False


def gps_reader():
   global gps
   socket = socket_map['gps']
   rt = RateTimer(1)
   while True:
      data = socket.recv()
      if rt.expired():
         gps = loads(data)


def cpu_reader():
   global load
   load = None
   while True:
      if load is None:
         load = cpu_percent()
      else:
         load = 0.8 * load + 0.2 * cpu_percent()
      sleep(0.1)


def pm_reader():
   s = socket_map['powerman']
   global voltage, current, critical
   while True:
      _voltage, _current, _, critical = loads(s.recv())
      if voltage is None:
         voltage = _voltage
      else:
         voltage = 0.9 * voltage + 0.1 * _voltage
      if current is None:
         current = _current
      else:
         current = 0.9 * current + 0.1 * _current


def mem_used():
   lines = file('/proc/meminfo').readlines()
   total = float(lines[0].split(' ')[-2])
   free = float(lines[1].split(' ')[-2])
   cached = float(lines[3].split(' ')[-2])
   return int(100 * (1.0 - (free + cached) / total))


def main(name):
   global socket_map, font, caution_written
   socket_map = generate_map(name)
   t1 = Thread(target = cpu_reader)
   t1.daemon = True
   t1.start()

   t2 = Thread(target = pm_reader)
   t2.daemon = True
   t2.start()

   t3 = Thread(target = gps_reader)
   t3.daemon = True
   t3.start()

   socket = generate_map('aircomm_app')['aircomm_in']
   while True:
      try:
         data = [BCAST_NOFW, HEARTBEAT, int(voltage * 10), int(current * 10), int(load), mem_used(), critical]
         try:
            data += [gps[LAT], gps[LON]]
         except:
            pass
         socket.send(dumps(data))
      except Exception, e:
         print e
      sleep(1.0)


daemonize('heartbeat', main)

