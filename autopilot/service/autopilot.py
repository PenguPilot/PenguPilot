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
 
 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from activities.takeoff import TakeoffActivity
from activities.land import LandActivity
from activities.move import MoveActivity
from activities.stop import StopActivity
from activities.dummy import DummyActivity
from flightsm import FlightSM
from scl import scl_get_socket, SCL_Reader


class Autopilot:

   def __init__(self):
      self.fsm = FlightSM(self.error, self.broadcast, self.takeoff, self.land, self.move, self.stop)
      self.orientation = SCL_Reader('orientation', 'sub', [0.0] * 3)
      self.pse = SCL_Reader('pos_speed_est_neu', 'sub', [0.0] * 8)
      self.state_socket = scl_get_socket('ap_state', 'pub')
      self.act = DummyActivity()
      self.act.start()


   def error(self, state, event):
      msg = 'in state: %s, could not handle event: %s' % (state, event)
      raise Exception(msg)


   def broadcast(self, state):
      self.state_socket.send(state)


   def takeoff(self):
      self.act.cancel_and_join()
      self.act = TakeoffActivity(self.fsm, self)
      self.act.start()


   def land(self):
      self.act.cancel_and_join()
      self.act = LandActivity(self)
      self.act.start()


   def move(self):
      self.act.cancel_and_join()
      self.act = MoveActivity(self)
      self.act.start()


   def stop(self):
      self.act.cancel_and_join()
      self.act = StopActivity(self)
      self.act.start()


def main(name):
   ap = Autopilot()
   socket = scl_get_socket('ap_ctrl', 'sub')
   while True:
      cmd = socket.recv()
      print cmd
      if isinstance(cmd, str):
         arg = None
      else:
         arg = cmd[1:]
         cmd = cmd[0]
      print cmd, arg
      ap.arg = arg
      getattr(ap, cmd)()


main('autopilot')
#daemonize('autopilot', main)

