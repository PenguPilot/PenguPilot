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
from zmq import Context, SUB, SUBSCRIBE

assert len(argv) == 2
context = Context()
socket = context.socket(SUB)
socket.connect("ipc://" + argv[1])
socket.setsockopt(SUBSCRIBE, '')

try:
   while True:
      raw = socket.recv()
      try:
         print loads(raw)
      except:
         print raw
except:
   print 'canceled by user'
