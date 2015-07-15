#!/usr/bin/env python
"""
  __________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|

 SCL Live Dump of Msgpack Subscriptions
 the argument is: "/tmp/scl_[socket_name]"

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sys import argv
from msgpack import loads
from scl import scl_get_socket


assert len(argv) >= 2
socket = scl_get_socket(argv[1], 'sub')
try:
   while True:
      raw = socket.zmq_socket.recv()
      try:
         data = loads(raw)
         if len(argv) >= 3:
            data = data[int(argv[2]):]
         if len(argv) == 4:
            data = data[0:int(argv[3])]
         print data
      except Exception, e:
         print e
         print len(raw), raw
except:
   print 'canceled by user'

