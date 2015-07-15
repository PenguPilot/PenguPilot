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
  
 Remote Control Stick Gestures

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from misc import daemonize, Hysteresis
from scl import scl_get_socket
from time import time


def main(name):
   valid_hyst = Hysteresis(5.0)
   start_hyst = Hysteresis(0.5)
   stop_hyst = Hysteresis(0.5)
   rc_socket = scl_get_socket('rc', 'sub')
   rc_gestures_socket = scl_get_socket('rc_gestures', 'pub')
   start_motors_prev = True
   stop_motors_prev = True
   
   while True:
      start_motors = False
      stop_motors = False
      rc_data = rc_socket.recv()

      # evaluate signal validity:
      if rc_data[0]:
         valid = valid_hyst.set()
      else:
         valid = valid_hyst.reset()

      # evaluate gestures:
      if valid and rc_data[5]:
         pitch_stick, roll_stick, yaw_stick, gas_stick = rc_data[1:5]
         THRESH = 0.8
         if gas_stick < -THRESH and yaw_stick > THRESH:
            start_motors = start_hyst.set()
         else:
            start_hyst.reset()
         if gas_stick < -THRESH and yaw_stick < -THRESH:
            stop_motors = stop_hyst.set()
         else:
            stop_hyst.reset()
      else:
         start_hyst.reset()
         stop_hyst.reset()

      # send gesture detection results:
      if start_motors != start_motors_prev or stop_motors != stop_motors_prev:
         rc_gestures_socket.send([start_motors, stop_motors])
      start_motors_prev = start_motors
      stop_motors_prev = stop_motors


daemonize('rc_gestures', main)

