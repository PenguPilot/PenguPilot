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

from logging import debug as log_debug, info as log_info, warning as log_warn, error as log_err



class TakeoffActivity(Activity, StabMixIn):

   LOW_ALT_SETPOINT = -10.0
   STD_HOVERING_ALT = 3.0


   def __init__(self, fsm, icarus):
      Activity.__init__(self, icarus)
      self.canceled = False
      self.fsm = fsm


   def _cancel(self):
      self.canceled = True


   def run(self):
      arg = self.icarus.arg
      pilot = self.icarus.pilot
      mon_data = self.icarus.mon_data
      params = self.icarus.pilot.params

      if arg.HasField('move_data'):
         z_setpoint = arg.move_data.z
         if arg.HasField('rel'):
            log_warn('rel field ignored for take-off')
         if arg.HasField('glob'):
            if not arg.glob:
               if z_setpoint < pilot.params.start_alt + mon_data.z + 3.0:
                  msg = 'absolute z setpoint %f is below current altitude' % z_setpoint
                  log_err(msg)
                  raise ValueError(msg)
               log_info('taking off to absolute altitude %f' % z_setpoint)
            else:
               z_setpoint = mon_data.z + z_setpoint
               log_info('taking off to relative altitude %f' % z_setpoint)
      else:
         z_setpoint = self.STD_HOVERING_ALT

      pilot.spin_up()

      if self.canceled:
         pilot.spin_down()
         log_error('take-off canceled');
         return

      # "point of no return":
      # reset controllers:
      pilot.set_ctrl_param(POS_YAW, mon_data.yaw)
      pilot.set_ctrl_param(POS_E, mon_data.e)
      pilot.set_ctrl_param(POS_N, mon_data.n)
      pilot.reset_ctrl()

      # set new altitude setpoint and stabilize:
      pilot.set_ctrl_param(POS_U, u_setpoint)
      self.stabilize()
      self.fsm.handle('done')

