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
 
 Takeoff Activity Class

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from pilot_pb2 import *
from activity import Activity, StabMixIn
from time import sleep
from logging import debug as log_debug, info as log_info, warning as log_warn, error as log_err



class TakeoffActivity(Activity, StabMixIn):


   def __init__(self, fsm, icarus):
      Activity.__init__(self, icarus)
      self.canceled = False
      self.fsm = fsm


   def _cancel(self):
      self.canceled = True


   def run(self):
      arg = self.icarus.arg
      pilot = self.icarus.pilot

      if arg.HasField('move_data'):
         u_max = 3.5
         if arg.move_data.z > u_max:
            u_setpoint = u_max
         else:
            u_setpoint = arg.move_data.z
      else:
         u_setpoint = 1.0

      u_setpoint = 4.0
      pilot.start_motors()

      if self.canceled:
         pilot.stop_motors()
         log_error('take-off canceled');
         return

      # "point of no return":
      # reset controllers:
      pilot.set_ctrl_param(POS_N, pilot.mon[0])
      pilot.set_ctrl_param(POS_E, pilot.mon[1])
      pilot.set_ctrl_param(POS_YAW, pilot.mon[4])
      pilot.reset_ctrl()

      # set new altitude setpoint and stabilize:
      u_setp = -1.0
      while u_setp < u_setpoint:
         pilot.set_ctrl_param(POS_U_GROUND, u_setp)
         u_setp += 0.05
         sleep(0.1)
      pilot.set_ctrl_param(POS_U_GROUND, u_setpoint)
      self.stabilize()
      self.fsm.handle('done')

