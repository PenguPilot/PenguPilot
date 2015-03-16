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

 SCL Python interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import os, zmq


try:
   pp_path = "ipc://" + os.environ["HOME"] + "/.PenguPilot/ipc/"
except:
   raise ZSEx("HOME environment variable is not set")
context = zmq.Context()


def scl_get_socket(id, type_name):
   map = {"sub": zmq.SUB, "req": zmq.REQ, "push": zmq.PUSH,
          "pub": zmq.PUB, "rep": zmq.REP, "pull": zmq.PULL}
   socket_type = map[type_name]
   socket_path = pp_path + id
   socket = context.socket(socket_type)
   if socket_type in [zmq.SUB, zmq.REQ, zmq.PUSH]:
      if socket_type == zmq.SUB:
         socket.setsockopt(zmq.SUBSCRIBE, "")
      socket.connect(socket_path)
   elif socket_type in [zmq.PUB, zmq.REP, zmq.PULL]:
      socket.bind(socket_path)
   else:
      raise Exception("unknown socket type: %d" % socket_type)
   return socket

