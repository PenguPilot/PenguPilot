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
from power_pb2 import PowerState
from msgpack import dumps
from aircomm import BCAST, HEARTBEAT

socket_map = None


def gps():
   global gps_data, socket_map
   socket = socket_map['gps']
   i = 0
   while True:
      with gps_lock:
         data = socket.recv()
         if i == 5:
            i = 0
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
   s = socket_map['power']
   p = PowerState()
   global voltage
   voltage = None
   while True:
      p.ParseFromString(s.recv())
      if voltage is None:
         voltage = p.voltage
      else:
         voltage = 0.9 * voltage + 0.1 * p.voltage


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

   socket = socket_map['out']
   while True:
      try:
         data = [BCAST, HEARTBEAT, int(voltage * 10), int(load), mem_used()]
         with gps_lock:
            data += [gps_data.fix, gps_data.sats]
            if gps_data.fix >= 2:
               data += [gps_data.lon, gps_data.lat]
         print len(dumps(data))
         socket.send(dumps(data))
         sleep(1)
      except Exception, e:
         sleep(1)

daemonize('heartbeat', main)

