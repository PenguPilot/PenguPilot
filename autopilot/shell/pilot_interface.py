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
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from pilot_pb2 import *


class AutopilotError(Exception):

   def __init__(self, status, err_msg):
      self.status = status
      self.err_msg = err_msg

   def __repr__(self):
      err_map = {E_SYNTAX: 'E_SYNTAX', E_SEMANTIC: 'E_SEMANTIC', E_HARDWARE: 'E_HARDWARE'}
      return 'class: ' + err_map[self.status] + ' message: ' + self.err_msg


class PilotInterface:

   def __init__(self, ctrl_socket, mon_socket):
      self.ctrl_socket = ctrl_socket
      self.params = self.get_params()
      self.mon_socket = mon_socket

   def _exec(self, req):
      self.ctrl_socket.send(req.SerializeToString())
      rep = AutopilotRep()
      rep.ParseFromString(self.ctrl_socket.recv())
      if rep.status != 0:
         raise AutopilotError(rep.status, rep.err_msg)
      return rep

   def mode_normal(self):
      req = AutopilotReq()
      req.type = MODE_NORMAL
      self._exec(req)

   def mode_cal(self):
      req = AutopilotReq()
      req.type = MODE_CAL
      self._exec(req)

   def spin_up(self):
      req = AutopilotReq()
      req.type = SPIN_UP
      self._exec(req)

   def spin_down(self):
      req = AutopilotReq()
      req.type = SPIN_DOWN
      self._exec(req)

   def reset_ctrl(self):
      req = AutopilotReq()
      req.type = RESET_CTRL
      self._exec(req)
  
   def set_ctrl_param(self, param, val):
      req = AutopilotReq()
      req.type = SET_CTRL_PARAM
      req.ctrl_data.param = param
      req.ctrl_data.val = val
      self._exec(req)

   def get_params(self):
      req = AutopilotReq()
      req.type = GET_PARAMS
      rep = self._exec(req)
      return rep.params

   def mon_read(self, mon):
      data = self.mon_socket.recv()
      mon.ParseFromString(data)

