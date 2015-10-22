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

 
 Control System API

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


class _OutputEnable:

   def __init__(self, sockets):
      self.sockets = sockets
      self.prev_state = None

   def set(self, state):
      assert state == 0 or state == 1
      if self.prev_state != state:
         for s in self.sockets:
            s.send(state)
      self.prev_state = state



class CtrlAPI:

   def __init__(self):
      # thrust and thrust maximum:
      self._thrust = scl_get_socket('thrust_p', 'push')
      self._thrust_max = scl_get_socket('thrust_maxp', 'push')

      # vertical speed control:
      self._vs_sp = scl_get_socket('vs_ctrl_spp', 'push')
      self._vs_oe_ = scl_get_socket('vs_ctrl_oe', 'push')
      self._vs_oe = _OutputEnable([self._vs_oe_])

      # vertical position control:
      self._vp_oe_ = scl_get_socket('vp_ctrl_oe', 'push')
      self._vp_oe = _OutputEnable([self._vp_oe_])
      self._vp_sp = scl_get_socket('vp_ctrl_spp', 'push')

      # horizontal speed control:
      self._hs_p_oe = scl_get_socket('hs_ctrl_p_oe', 'push')
      self._hs_r_oe = scl_get_socket('hs_ctrl_r_oe', 'push')
      self._hs_oe = _OutputEnable([self._hs_p_oe, self._hs_r_oe])
      self._hs_sp_n = scl_get_socket('hs_ctrl_spp_n', 'push')
      self._hs_sp_e = scl_get_socket('hs_ctrl_spp_e', 'push')

      # horizontal position control:
      self._hp_n_oe = scl_get_socket('hp_ctrl_n_oe', 'push')
      self._hp_e_oe = scl_get_socket('hp_ctrl_e_oe', 'push')
      self._hp_oe = _OutputEnable([self._hp_n_oe, self._hp_e_oe])
      self._hp_sp_n = scl_get_socket('hp_ctrl_spp_n', 'push')
      self._hp_sp_e = scl_get_socket('hp_ctrl_spp_e', 'push')

      # rotation position control:
      self._rp_p_oe = scl_get_socket('rp_ctrl_p_oe', 'push')
      self._rp_r_oe = scl_get_socket('rp_ctrl_r_oe', 'push')
      self._rp_y_oe_ = scl_get_socket('rp_ctrl_y_oe', 'push')
      self._rp_oe = _OutputEnable([self._rp_p_oe, self._rp_r_oe])
      self._rp_y_oe = _OutputEnable([self._rp_y_oe_])
      self._rp_sp_p = scl_get_socket('rp_ctrl_spp_p', 'push')
      self._rp_sp_r = scl_get_socket('rp_ctrl_spp_r', 'push')
      self._rp_sp_y = scl_get_socket('rp_ctrl_spp_y', 'push')

      # rotation speed control:
      self._rs_sp_p = scl_get_socket('rs_ctrl_spp_p', 'push')
      self._rs_sp_r = scl_get_socket('rs_ctrl_spp_r', 'push')
      self._rs_sp_y = scl_get_socket('rs_ctrl_spp_y', 'push')
      self._rs_oe_ = scl_get_socket('rs_ctrl_oe', 'push')
      self._rs_oe = _OutputEnable([self._rs_oe_])

      # torques:
      self._torques = scl_get_socket('torques_p', 'push')

      # motor enable:
      self._mot_en = scl_get_socket('mot_en', 'push')


   def mot_en(self, val):
      """enables (True) or disables (False) the motors"""
      self._mot_en.send(int(val))


   def set_ys(self, val):
      """sets the yaw speed in rad/s"""
      self._rp_y_oe.set(0)
      self._rs_sp_y.send(float(val))


   def set_yp(self, val):
      """sets yaw position in rad, North direction is 0, rotating right positive"""
      self._rp_y_oe.set(1)
      self._rp_sp_y.send(float(val))


   def set_thrust(self, val):
      """sets the overall thrust value in Newtons"""
      self._vs_oe.set(0)
      self._thrust.send(float(val))


   def set_thrust_max(self, val):
      """sets upper thrust limit, used as a safety feature in autonomous flight"""
      self._thrust_max.send(float(val))


   def set_vs(self, val):
      """sets vertical speed based on barometer speed estimate"""
      self._vs_oe.set(1)
      self._vp_oe.set(0)
      self._vs_sp.send(float(val))


   def set_vp(self, val, mode = 'ultra'):
      """sets the vertical position with mode in:
         - ultra: ultrasonic height above ground
         - baro_abs: baro above MSL
         - baro_rel: baro relative to power-up starting value
         - evel: baro control accoding to SRTM elevation map (val: safety distance)"""
      self._vs_oe.set(1)
      self._vp_oe.set(1)
      self._vp_sp.send([str(mode), float(val)])


   def set_torques(self, vec):
      """sets 3D torques directly, without control; this is intended for debugging or calibration, not for flying!"""
      self._rs_oe.set(0)
      vec = map(float, vec)
      self._torques.send(vec)


   def set_rs(self, vec):
      """sets the rotation speed of pitch and roll"""
      self._rs_oe.set(1)
      self._rp_oe.set(0)
      self._rs_sp_p.send(float(vec[0]))
      self._rs_sp_r.send(float(vec[1]))


   def set_rp(self, vec):
      """sets the rotation position of pitch and roll"""
      self._rs_oe.set(1)
      self._rp_oe.set(1)
      self._hs_oe.set(0)
      self._hp_oe.set(0)
      self._rp_sp_p.send(float(vec[0]))
      self._rp_sp_r.send(float(vec[1]))


   def set_hs(self, vec):
      """sets horizontal speed in north/east direction in meters per second"""
      self._rs_oe.set(1)
      self._rp_oe.set(1)
      self._hs_oe.set(1)
      self._hp_oe.set(0)
      self._hs_sp_n.send(float(vec[0]))
      self._hs_sp_e.send(float(vec[1]))


   def set_hp(self, vec):
      """sets horizontal position in north/east direction in meters (relative coordinates according to first GPS fix)"""
      self._rs_oe.set(1)
      self._rp_oe.set(1)
      self._hs_oe.set(1)
      self._hp_oe.set(1)
      self._hp_sp_n.send(float(vec[0]))
      self._hp_sp_e.send(float(vec[1]))

