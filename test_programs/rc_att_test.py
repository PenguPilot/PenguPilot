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


from scl import scl_get_socket
from msgpack import dumps, loads
from time import sleep

rc_socket = scl_get_socket('rc_cal', 'sub')

rs_ctrl_sp_p_socket = scl_get_socket('rp_ctrl_sp_p', 'pub')
rs_ctrl_sp_r_socket = scl_get_socket('rp_ctrl_sp_r', 'pub')

rs_ctrl_sp_y_socket = scl_get_socket('rs_ctrl_spp_y', 'push')
thrust_socket = scl_get_socket('thrust', 'pub')
mot_en_socket = scl_get_socket('mot_en', 'pub')
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
        
         # send 3 rate setpoints
         if abs(pitch) < 0.05: pitch = 0.0
         if abs(roll) < 0.05: roll = 0.0
         if abs(yaw) < 0.05: yaw = 0.0

         rs_ctrl_sp_p_socket.send(dumps(-0.35 * pitch))
         rs_ctrl_sp_r_socket.send(dumps(0.35 * roll))
         rs_ctrl_sp_y_socket.send(dumps(0.6 * yaw))
      else:
          mot_en_socket.send(dumps(0))
except:
   pass

mot_en_socket.send(dumps(0))
sleep(0.5)
