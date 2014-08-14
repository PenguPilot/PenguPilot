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
  
 Online Parameter Configuration Daemon

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from config import Config, ConfigError
from opcd_pb2 import CtrlReq, CtrlRep, Pair
from scl import generate_map
from misc import daemonize
from sys import argv
from re import match
from time import sleep
from pylogger import *


class OPCD:


   def __init__(self, name):
      map = generate_map(name)
      self.ctrl_socket = map['opcd_ctrl']
      self.event_socket = map['opcd_event']
      logger_init('opcd', map['log_data'])
      self.conf = Config()
      self.map = {str: 'str_val', int: 'int_val', float: 'dbl_val', bool: 'bool_val'}


   def _pairs_add(self, key, rep):
      try:
         val = self.conf.get(key)
         pair = rep.pairs.add()
         pair.id = key
         setattr(pair.val, self.map[val.__class__], val)
      except ConfigError:
         rep.status = CtrlRep.PARAM_UNKNOWN
      except ValueError: # TODO: check: does this still apply?
         rep.status = CtrlRep.MALFORMED_ID


   def run(self):

      while True:
         # read and parse request:
         req = CtrlReq()
         try:
            req.ParseFromString(self.ctrl_socket.recv())
         except Exception, e:
            log(LL_ERROR, 'could not receive request: %s' % str(e))
            sleep(1)
            continue
         # process request and prepare reply:
         rep = CtrlRep()
         rep.status = CtrlRep.OK

         try:
            # GET REQUEST:
            if req.type == CtrlReq.GET:
               all_keys = self.conf.get_all_keys(self.conf.base) + ['platform']
               if req.id in all_keys:
                  # exact match:
                  self._pairs_add(req.id, rep)
               else:
                  # try regex matches:
                  found = False
                  for key in all_keys:
                     try:
                        if match(req.id, key):
                           self._pairs_add(key, rep)
                           found = True
                     except:
                        pass
                  if not found:
                     log(LL_ERROR, 'key not found: %s' % req.id)
                     rep.status = CtrlRep.PARAM_UNKNOWN

            # SET REQUEST:
            elif req.type == CtrlReq.SET:
               try:
                  for type, attr in self.map.items():
                     if req.val.HasField(attr):
                        val = getattr(req.val, attr)
                        self.conf.set(req.id.encode('ascii'), type(val))
                        break
                  pair = Pair(id = req.id, val = req.val)
                  self.event_socket.send(pair.SerializeToString())
               except ConfigError, e:
                  log(LL_ERROR, 'key not found: %s' % req.id)
                  rep.status = CtrlRep.PARAM_UNKNOWN

            # PERSIST REQUEST:
            else:
               assert req.type == CtrlReq.PERSIST
               try:
                  self.conf.persist()
                  rep.status = CtrlRep.OK
               except Exception, e:
                  log(LL_ERROR, 'persist failed: %s' % str(e))
                  rep.status = CtrlRep.IO_ERROR
         except:
            rep.status = CtrlRep.PARAM_UNKNOWN
         # send reply:
         self.ctrl_socket.send(rep.SerializeToString())


def main(name):
   opcd = OPCD(name)
   opcd.run()


daemonize('opcd', main)

