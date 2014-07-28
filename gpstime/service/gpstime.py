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
  
 GPS Timezone/Time Setting Service

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from scl import generate_map
from misc import daemonize
from gps_msgpack import *
from msgpack import loads
from os import system
from tzwhere.tzwhere import tzwhere


def main(name):
   tzw = tzwhere()
   socket = generate_map(name)
   while True:
      try:
         raw = socket.recv()
         gps = loads(raw)
         time, lat, lon = gps[time], gps[LAT], gps[LON]
         break
      except:
         pass
   tz_str = tzw.tzNameAt(lat, lon)
   # update time zone:
   system('cp /usr/share/zoneinfo/%s /etc/localtime' % tz_str)
   # set data & time:
   system('date -s "%s"' % gps[TIME])


daemonize('gpstime', main)

