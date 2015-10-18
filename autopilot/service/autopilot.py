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
from pylogger import *
from misc import daemonize
from ctrl_api import CtrlAPI

class Autopilot:

   def __init__(self):
      self.api = CtrlAPI()
      self.fsm = FlightSM(self.error, self.broadcast, self.takeoff, self.land, self.move, self.stop)
      self.orientation = SCL_Reader('orientation', 'sub', [0.0] * 3)
      self.pse = SCL_Reader('pos_speed_est_neu', 'sub', [0.0] * 8)
      self.state_socket = scl_get_socket('ap_state', 'pub')
      self.act = DummyActivity()
      self.act.start()


   def error(self, state, event):
      msg = 'invalid event "%s" in state "%s"' % (event, state)
      log(LL_ERROR, msg)
      raise Exception(msg)


   def broadcast(self, state):
      log(LL_INFO, 'new state: %s' % state)
      self.state_socket.send(state)


   def takeoff(self):
      log(LL_INFO, 'takeoff')
      self.act.cancel_and_join()
      self.act = TakeoffActivity(self.fsm, self)
      self.act.run()


   def land(self):
      log(LL_INFO, 'land')
      self.act.cancel_and_join()
      self.act = LandActivity(self)
      self.act.run()


   def move(self):
      log(LL_INFO, 'move')
      self.act.cancel_and_join()
      self.act = MoveActivity(self)
      self.act.run()


   def stop(self):
      log(LL_INFO, 'stop')
      self.act.cancel_and_join()
      self.act = StopActivity(self)
      self.act.run()


   def handle(self, cmd):
      if isinstance(cmd, str):
         arg = None
      else:
         arg = cmd[1:]
         cmd = cmd[0]
      self.arg = arg
      self.fsm.handle(cmd)
      #after execution, DummyActivity needs to be started again
      self.act = DummyActivity()
      self.act.start()


def main(name):
   ap = Autopilot()
   ap.motors_state = scl_get_socket('mot_state', 'sub')
   ap_ctrl = scl_get_socket('ap_ctrl', 'rep')
   while True:
      cmd = ap_ctrl.recv()
      try:
         ap.handle(cmd)
         ap_ctrl.send([True])
      except Exception, e:
         ap_ctrl.send([False, str(e)])


daemonize('autopilot', main, fg = False)

