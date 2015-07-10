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


from sticks import sticks_expo
from scl import scl_get_socket, SCL_Reader
from msgpack import dumps, loads
from time import sleep
from geomath import vec2_rot
from control_api import *
from gps_msgpack import fix


def channel_to_mode(sw):
   a = 1.0 / 3.0
   b = 2.0 / 3.0
   if sw <= a:
      return 'gyro'   
   elif sw > a and sw < b:
      return 'acc'
   return 'gps'


orientation = SCL_Reader('orientation', 'sub', [0.0])
rc_socket = scl_get_socket('rc', 'sub')
# rotation speed control:
rs_sp_y = scl_get_socket('rs_ctrl_spp_y', 'push')
thrust_socket = scl_get_socket('thrust', 'pub')
mot_en_socket = scl_get_socket('mot_en', 'push')
gps = SCL_Reader('gps', 'sub', [0])

sleep(1)
try:
   mode_prev = None
   while True:
      rc_data = loads(rc_socket.recv())
      if rc_data[0]:
         pitch_stick, roll_stick, yaw_stick, gas_stick, kill_switch, mode_switch = rc_data[1:7]

         if abs(pitch_stick) < 0.05: pitch_stick = 0.0
         if abs(roll_stick) < 0.05: roll_stick = 0.0
         if abs(yaw_stick) < 0.05: yaw_stick = 0.0
         
         if kill_switch > 0.5:
            mot_en_socket.send(dumps(1))
         else:
            mot_en_socket.send(dumps(0))

         # send gas and yaw value:
         thrust_socket.send(dumps(10.0 * (gas_stick + 1.0)))
         rs_sp_y.send(dumps(0.6 * yaw_stick))

         mode = channel_to_mode(mode_switch)
         if mode == 'gps' and fix(gps.data) < 2:
            mode = 'acc'
         if mode_prev != mode:
            print 'new mode:', mode
         mode_prev = mode
         if mode == 'gps':
            scale = 3.0
            v_local = [scale * pitch_stick, scale * roll_stick]
            v_global = vec2_rot(v_local, orientation.data[0])
            set_hs(v_global)
         elif mode == 'acc':
            scale = 0.45
            set_rp([-scale * pitch_stick, scale * roll_stick])
         else: # gyro
            scale = 1.0
            set_rs([-scale * pitch_stick, scale * roll_stick])
      else:
         mot_en_socket.send(dumps(0))
except Exception, e:
   print e

mot_en_socket.send(dumps(0))
sleep(0.5)
