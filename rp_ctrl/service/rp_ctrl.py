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
  
 Pitch/Roll/Yaw Rotation Position Control

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
from opcd_interface import OPCD_Subscriber
from geomath import deg2rad, sym_limit, angles_diff
from misc import daemonize


def main(name):
   # start opcd subscriber:
   opcd = OPCD_Subscriber()

   # setpoint readers:
   sp_p = SCL_Reader('rp_ctrl_sp_p', 'sub')
   sp_r = SCL_Reader('rp_ctrl_sp_r', 'sub')
   sp_y = SCL_Reader('rp_ctrl_sp_y', 'sub')

   # enable readers:
   p_oe = SCL_Reader('rp_ctrl_p_oe', 'pull')
   p_oe.data = 1 # enabled, if nothing received

   r_oe = SCL_Reader('rp_ctrl_r_oe', 'pull')
   r_oe.data = 1 # enabled, if nothing received

   y_oe = SCL_Reader('rp_ctrl_y_oe', 'pull')
   y_oe.data = 0 # disabled, if nothing received (why? we need to be careful with the setpoint,
                 # in contrast to pitch and roll, which are safe at 0 value


   # outgoing sockets:
   spp_p_socket = scl_get_socket('rs_ctrl_spp_p', 'push')
   spp_r_socket = scl_get_socket('rs_ctrl_spp_r', 'push')
   spp_y_socket = scl_get_socket('rs_ctrl_spp_y', 'push')


   orientation_socket = scl_get_socket('orientation', 'sub')
   while True:
      yaw, pitch, roll = loads(orientation_socket.recv())[0:3]
      pr_p = opcd['rp_ctrl.pr_p']
      yaw_p = opcd['rp_ctrl.yaw_p']
      pitch_bias = deg2rad(opcd['rp_ctrl.pitch_bias'])
      roll_bias = deg2rad(opcd['rp_ctrl.roll_bias'])
      angles_max = deg2rad(opcd['rp_ctrl.angles_max'])

      # pitch position control:
      pitch_err = pitch - sym_limit(sp_p.data, angles_max) + pitch_bias
      if p_oe.data:
         spp_p_socket.send(dumps(-pitch_err * pr_p))

      # roll position control:
      roll_err = roll - sym_limit(sp_r.data, angles_max) + roll_bias
      if r_oe.data:
         spp_r_socket.send(dumps(-roll_err * pr_p))
          
      # yaw position control:
      yaw_err = angles_diff(yaw, sp_y.data)
      if y_oe.data:
         spp_y_socket.send(dumps(yaw_err * yaw_p))

daemonize('rp_ctrl', main)
