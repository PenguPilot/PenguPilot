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
  
 Mixer Test Utility

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from scl import scl_get_socket, SCL_Reader
from msgpack import dumps, loads
from time import sleep
from geomath import vec2_rot

orientation = SCL_Reader('orientation', 'sub', [0.0])
rc_socket = scl_get_socket('rc', 'sub')

n_socket = scl_get_socket('hs_ctrl_spp_n', 'push')
e_socket = scl_get_socket('hs_ctrl_spp_e', 'push')
yaw_speed_socket = scl_get_socket('rs_ctrl_spp_y', 'push')

thrust_socket = scl_get_socket('thrust', 'pub')
mot_en_socket = scl_get_socket('mot_en', 'push')
sleep(1)

try:
   while True:
      rc_data = loads(rc_socket.recv())
      if rc_data[0]:
         pitch, roll, yaw, gas, sw = rc_data[1:6]
         if sw > 0.5:
            mot_en_socket.send(dumps(1))
         else:
            mot_en_socket.send(dumps(0))
         
         # send gas value:
         thrust_socket.send(dumps(10.0 * (gas + 1)))
        
         # deadzone for setpoints:
         if abs(pitch) < 0.05: pitch = 0.0
         if abs(roll) < 0.05: roll = 0.0
         if abs(yaw) < 0.05: yaw = 0.0

         # rotate setpoints:
         v_local = [3.0 * pitch, 3.0 * roll]
         v_global = vec2_rot(v_local, orientation.data[0])
         
         # send global speed setpoints:
         n_socket.send(dumps(v_global[0]))
         e_socket.send(dumps(v_global[1]))
         yaw_speed_socket.send(dumps(0.6 * yaw))
      else:
         mot_en_socket.send(dumps(0))
except Exception, e:
   print e

mot_en_socket.send(dumps(0))
sleep(0.5)
