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
  
 PowerMan Interface

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from power_pb2 import *


class PowerException(Exception):
   pass


class PowerMan:
   
   def __init__(self, ctrl_socket, mon_socket):
      self.ctrl_socket = ctrl_socket
      self.mon_socket = mon_socket

   def _exec(self, cmd):
      req = PowerReq()
      req.cmd = cmd
      self.ctrl_socket.send(req.SerializeToString())
      rep = PowerRep()
      rep.ParseFromString(self.ctrl_socket.recv())
      if rep.status != OK:
         if rep.status == E_SYNTAX:
            print 'received reply garbage'
         else:
            raise PowerException

   def stand_power(self):
      self._exec(STAND_POWER)

   def flight_power(self):
      self._exec(FLIGHT_POWER)

   def read(self):
      state = PowerState()
      data = self.mon_socket.recv()
      state.ParseFromString(data)
      return state

