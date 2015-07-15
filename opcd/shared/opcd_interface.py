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
  
 OPCD Python Binding

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from opcd_pb2 import CtrlReq, CtrlRep, Pair
from threading import Thread, Lock
from scl import scl_get_socket


fields_map = {str: 'str_val', int: 'int_val', float: 'dbl_val', bool: 'bool_val'}


def pair_get_val(pair):
   for type in fields_map.values():
      if pair.val.HasField(type):
         val = getattr(pair.val, type)
         if type == 'str_val':
            val = val.encode('ascii')
         return val


class OPCD_Subscriber(Thread):

   def __init__(self):
      global subscriber
      Thread.__init__(self)
      self.pairs = dict(OPCD_Interface().get(''))
      self.daemon = True
      self.socket = scl_get_socket('opcd_event', 'sub')
      _subscriber = self
      self.start()


   def run(self):
      while True:
         pair = Pair()
         data = self.socket.zmq_socket.recv()
         pair.ParseFromString(data)
         self.pairs[pair.id] = pair_get_val(pair)

   def __getitem__(self, name):
      return self.pairs[name]


class OPCD_Interface:


   def __init__(self, prefix = None):
      self.socket = scl_get_socket('opcd_ctrl', 'req')
      self.prefix = prefix
      self.lock = Lock()

   def _send_and_recv(self, req):
      self.lock.acquire()
      self.socket.zmq_socket.send(req.SerializeToString())
      rep = CtrlRep()
      rep.ParseFromString(self.socket.zmq_socket.recv())
      self.lock.release()
      return rep


   def get(self, id, return_list = False):
      req = CtrlReq()
      req.type = CtrlReq.GET
      if self.prefix:
         id = self.prefix + '.' + id
      req.id = id
      rep = self._send_and_recv(req)
      if rep.status != 0:
         raise KeyError(id)
      pairs = []
      for pair in rep.pairs:
         pairs.append((pair.id.encode('ascii'), pair_get_val(pair)))
      if len(pairs) == 0:
         return
      elif len(pairs) == 1:
         if return_list:
            return pairs[0]
         else:
            return pairs[0][1]
      else:
         return pairs


   def set(self, id, val):
      req = CtrlReq()
      req.type = CtrlReq.SET
      if self.prefix:
         id = self.prefix + '.' + id
      req.id = id
      setattr(req.val, fields_map[val.__class__], val)
      rep = self._send_and_recv(req)
      if rep.status != 0:
         raise KeyError(id)


   def persist(self):
      req = CtrlReq()
      req.type = CtrlReq.PERSIST
      return self._send_and_recv(req).status

