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
  
 Semi Auto Flight Logic

 Copyright (C) Integrated Communication Systems Group, TU Ilmenau

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
from time import sleep
from geomath import vec2_rot
from ctrl_api import CtrlAPI
from gps_msgpack import fix
from opcd_interface import OPCD_Subscriber
from pos_speed_est_neu import N_POS, E_POS
from pid_ctrl import PID_Ctrl
import math
import time

def mot_en_cb(gesture):
   if gesture[0]: # start
      api.mot_en(True)
      print 'start'
   if gesture[1]: # stop
      api.mot_en(False)
      print 'stop'

orientation = SCL_Reader('orientation', 'sub', [0.0])
pse = SCL_Reader('pos_speed_est_neu', 'sub')
flying = SCL_Reader('flying', 'sub', 0)
rc_socket = scl_get_socket('rc', 'sub')
gps = SCL_Reader('gps', 'sub', [0])
ultra_raw = SCL_Reader('ultra_raw', 'sub')
ap_ctrl = scl_get_socket('ap_ctrl', 'req')
ap_state = SCL_Reader('ap_status', 'sub')
#SCL_Reader('rc_gestures', 'sub', callback = mot_en_cb)

init(OPCD_Subscriber())
sleep(1)
api = CtrlAPI()
api.set_thrust_max(100.0)
try:
   killed = False
   mc = False
   pos_locked = False
   alt_locked = False
   mode_prev = None

   while True:
      rc_data = rc_socket.recv()
      state = ap_state.data      
      if rc_data[0]:
         pitch_stick, roll_stick, yaw_stick, gas_stick, on_switch, mode_switch = rc_data[1:7]
         mot_en_state = on_switch > 0.5
         api.mot_en(on_switch > 0.5)
         pr_sticks = [pitch_stick, roll_stick]
         
	 #set the mode of flight to a desired one
         mode = channel_to_mode(mode_switch)
         #if mode == 'gps' and fix(gps.data) < 2:
         #   mode = 'acc'
         if mode_prev != mode:
            print 'new mode:', mode
            once = 1
            init_vp = False
            task_vp = ''
            if mode == 'acc':
                print 'auto mode'
            if mode == 'gps':
                print 'manual mode'
            if mode == 'gyro':
                print '*** mode'

         if mode == 'gps':
            #full manual mode
            api.set_thrust(10.0 * (gas_stick + 1.0))
            api.set_ys(0.6 * yaw_stick)
              
            vals = map(pitch_roll_angle_func, pr_sticks)
            api.set_rp([-vals[0], vals[1]])
            
            if killed == False:
                #kill autopilot
                ap_ctrl.send('kill')
                killed = True
         
         if mode == 'gyro':
             #position hold/control, vertical control and takeoff/land via: autopilot
             if state == 'standing' and yaw_stick >= 0.7:
                 try:
                    ap_ctrl.send('takeoff')
                    rep = ap_ctrl.recv()
                    if not rep[0]:
                      print rep[1]
                 except Exception, e:
                    print e
                    pass

             if state == 'hovering' and yaw_stick <= -0.7:
                 try:
                    ap_ctrl.send('land')
                    rep = ap_ctrl.recv()
                    if not rep[0]:
                      print rep[1]
                 except Exception, e:
                    print e
                    pass
             if state == 'hovering' and yaw_stick > -0.7 and yaw_stick < 0.7:
                 #set the vertical position
                 vp = 2.5 * (gas_stick + 1.0)
                 if not alt_locked:
                     alt_locked = vp
                     if vp >= 1.0:
                        api.set_vp(vp, 'ultra')
                     else:
                        vp = 1.0
                        api.set_vp(vp, 'ultra')
                     print 'Desired altitude set: ' + str(vp)
                 else:
                     if abs(vp - alt_locked) >= 0.1:
                         alt_locked = False

                 #set horizontal speed using sticks, otherwise - hold pos
                 if pitch_roll_in_deadzone(pitch_stick, roll_stick):
                    if not pos_locked:
                        pos_locked = pse.data[N_POS], pse.data[E_POS]
                        print 'locking:', pos_locked
                        api.set_hp(pos_locked)
                 else:
                    pos_locked = None
                    scale = 3.0
                    v_local = [scale * pitch_stick, scale * roll_stick]
                    v_global = vec2_rot(v_local, orientation.data[0])
                    api.set_hs(v_global)

         if mode == 'acc':
             #autopilot controlled mode
             killed = False
             
             #limit thrust using gas stick
             api.set_thrust_max(13.0 * (gas_stick + 1.0))
             vals = map(pitch_roll_angle_func, pr_sticks)
             
             #if stick was touched -> use manual control, but autopilot is running anyway
             if abs(pitch_stick) > 0.2 or abs(roll_stick) > 0.2:
                mc = True
             
             if mc == True:
                api.set_rp([-vals[0], vals[1]])
                 #api.set_hp([pse.data[4], pse.data[6]])

         mode_prev = mode
      else:
         api.mot_en(False)
except Exception, e:
   print e

api.mot_en(False)
sleep(0.5)
