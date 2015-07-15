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
from time import sleep
from opcd_interface import OPCD_Subscriber
from geomath import deg2rad, sym_limit, circle_err
from misc import daemonize
from pp_prio import PP_PRIO_2
from scheduler import sched_set_prio
from pid_ctrl import PID_Ctrl
from math import pi



def main(name):
   sched_set_prio(PP_PRIO_2)

   # start opcd subscriber:
   opcd = OPCD_Subscriber()

   # state input sockets/readers:
   orientation_socket = scl_get_socket('orientation', 'sub')
   gyro = SCL_Reader('gyro', 'sub', [0.0, 0.0, 0.0])

   # setpoint readers:
   sp_p = SCL_Reader(name + '_sp_p', 'sub')
   sp_r = SCL_Reader(name + '_sp_r', 'sub')
   sp_y = SCL_Reader(name + '_sp_y', 'sub')

   # enable readers:
   p_oe = SCL_Reader(name + '_p_oe', 'pull')
   p_oe.data = 1 # enabled, if nothing received
   r_oe = SCL_Reader(name + '_r_oe', 'pull')
   r_oe.data = 1 # enabled, if nothing received
   y_oe = SCL_Reader(name + '_y_oe', 'pull')
   y_oe.data = 0 # disabled, if nothing received (why? we need to be careful with the setpoint,
                 # in contrast to pitch and roll, which are safe at 0 value

   # outgoing sockets:
   spp_p_socket = scl_get_socket('rs_ctrl_spp_p', 'push')
   spp_r_socket = scl_get_socket('rs_ctrl_spp_r', 'push')
   spp_y_socket = scl_get_socket('rs_ctrl_spp_y', 'push')

   # create controllers:
   pid_p = PID_Ctrl()
   pid_r = PID_Ctrl()
   pid_y = PID_Ctrl(circle_err) # special error computation function

   while True:
      yaw, pitch, roll = orientation_socket.recv()
      pid_p.p = pid_r.p = opcd[name + '.pr_p']
      pid_p.d = pid_r.d = opcd[name + '.pr_d']
      pid_y.p = opcd[name + '.yaw_p']
      pitch_bias = deg2rad(opcd[name + '.pitch_bias'])
      roll_bias = deg2rad(opcd[name + '.roll_bias'])
      angles_max = deg2rad(opcd[name + '.angles_max'])

      # pitch position control:
      ctrl_p = pid_p.control(pitch, sym_limit(sp_p.data, angles_max) + pitch_bias, gyro.data[1])
      if p_oe.data:
         spp_p_socket.send(ctrl_p)
      
      # roll position control:
      ctrl_r = pid_r.control(roll, sym_limit(sp_r.data, angles_max) + roll_bias, gyro.data[0])
      if r_oe.data:
         spp_r_socket.send(ctrl_r)

      # yaw position control:
      ctrl_y = pid_y.control(yaw, sp_y.data)
      if y_oe.data:
         spp_y_socket.send(ctrl_y)

daemonize('rp_ctrl', main)
