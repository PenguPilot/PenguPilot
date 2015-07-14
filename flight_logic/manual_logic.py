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
  
 Manual Flight Logic

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sticks import *
from scl import scl_get_socket, SCL_Reader
from msgpack import dumps, loads
from time import sleep
from geomath import vec2_rot
from ctrl_api import *
from gps_msgpack import fix
from opcd_interface import OPCD_Subscriber


def channel_to_mode(sw):
   a = 1.0 / 3.0
   b = 2.0 / 3.0
   if sw <= a:
      return 'gyro'   
   elif sw > a and sw < b:
      return 'acc'
   return 'gps'


def mot_en_cb(gesture):
   if gesture[0]: # start
      mot_en(True)
      print 'start'
   if gesture[1]: # stop
      mot_en(False)
      print 'stop'

orientation = SCL_Reader('orientation', 'sub', [0.0])
rc_socket = scl_get_socket('rc', 'sub')
gps = SCL_Reader('gps', 'sub', [0])
SCL_Reader('rc_gestures', 'sub', callback = mot_en_cb)

init(OPCD_Subscriber())
sleep(1)
set_thrust_max(1000.0)
try:
   mode_prev = None
   while True:
      rc_data = loads(rc_socket.recv())
      if rc_data[0]:
         pitch_stick, roll_stick, yaw_stick, gas_stick, _, mode_switch = rc_data[1:7]
         pr_sticks = [pitch_stick, roll_stick]

         set_thrust(10.0 * (gas_stick + 1.0))
         set_ys(0.6 * yaw_stick)

         mode = channel_to_mode(mode_switch)
         if mode == 'gps' and fix(gps.data) < 2:
            mode = 'acc'
         if mode_prev != mode:
            print 'new mode:', mode
         mode_prev = mode
         
         # evaluate input based on mode:
         if mode == 'gps':
            scale = 3.0
            v_local = [scale * pitch_stick, scale * roll_stick]
            v_global = vec2_rot(v_local, orientation.data[0])
            set_hs(v_global)
         elif mode == 'acc':
            vals = map(pitch_roll_angle_func, pr_sticks)
            set_rp([-vals[0], vals[1]])
         else: # gyro
            vals = map(pitch_roll_speed_func, pr_sticks)
            set_rs([-vals[0], vals[1]])
      else:
         mot_en(False)
except:
   pass

mot_en(False)
sleep(0.5)
