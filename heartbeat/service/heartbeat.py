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
from scl import generate_map
from gps_data_pb2 import GpsData
from math import sin, cos, pi
from misc import daemonize
from msgpack import loads
from msgpack import Packer
from aircomm_shared import BCAST_NOFW, HEARTBEAT


socket_map = None
voltage = 17.0
current = 0.4
critical = False


def gps():
   global gps_data, socket_map
   socket = socket_map['gps']
   i = 0
   while True:
      data = socket.recv()
      if i == 5:
         i = 0
         with gps_lock:
            gps_data = GpsData()
            gps_data.ParseFromString(data)
      i += 1


def cpuavg():
   global load
   load = None
   while True:
      if load is None:
         load = cpu_percent()
      else:
         load = 0.8 * load + 0.2 * cpu_percent()
      sleep(0.1)


def pmreader():
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
   global socket_map, gps_lock, font, caution_written
   socket_map = generate_map(name)
   gps_lock = Lock()

   t1 = Thread(target = cpuavg)
   t1.daemon = True
   t1.start()

   t2 = Thread(target = pmreader)
   t2.daemon = True
   t2.start()

   t3 = Thread(target = gps)
   t3.daemon = True
   t3.start()

   socket = generate_map('aircomm_app')['aircomm_in']
   packer = Packer(use_single_float = True)
   while True:
      try:
         data = [BCAST_NOFW, HEARTBEAT, int(voltage * 10), int(current * 10), int(load), mem_used(), critical]
         with gps_lock:
            try:
               if gps_data.fix >= 2:
                  data += [gps_data.lon, gps_data.lat]
            except:
               pass
         socket.send(packer.pack(data))
      except Exception, e:
         print e
      sleep(1.0)

daemonize('heartbeat', main)

