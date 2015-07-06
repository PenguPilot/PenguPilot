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
from msgpack import dumps


class _OutputEnable:

   def __init__(self, sockets):
      self.sockets = sockets
      self.prev_state = None

   def set(self, state):
      assert state == 0 or state == 1
      if self.prev_state != state:
         for s in self.sockets:
            s.send(dumps(state))
      self.prev_state = state


# horizontal speed control:
_hs_p_oe = scl_get_socket('hs_ctrl_p_oe', 'push')
_hs_r_oe = scl_get_socket('hs_ctrl_r_oe', 'push')
_hs_oe = _OutputEnable([_hs_p_oe, _hs_r_oe])
_hs_sp_n = scl_get_socket('hs_ctrl_spp_n', 'push')
_hs_sp_e = scl_get_socket('hs_ctrl_spp_e', 'push')

# horizontal position control:
_hp_n_oe = scl_get_socket('hp_ctrl_n_oe', 'push')
_hp_e_oe = scl_get_socket('hp_ctrl_e_oe', 'push')
_hp_oe = _OutputEnable([_hp_n_oe, _hp_e_oe])
_hp_sp_n = scl_get_socket('hp_ctrl_spp_n', 'push')
_hp_sp_e = scl_get_socket('hp_ctrl_spp_e', 'push')

# rotation position control:
_rp_p_oe = scl_get_socket('rp_ctrl_p_oe', 'push')
_rp_r_oe = scl_get_socket('rp_ctrl_r_oe', 'push')
_rp_oe = _OutputEnable([_rp_p_oe, _rp_r_oe])
_rp_sp_p = scl_get_socket('rp_ctrl_spp_p', 'push')
_rp_sp_r = scl_get_socket('rp_ctrl_spp_r', 'push')

# rotation speed control:
_rs_p_oe = scl_get_socket('rs_ctrl_p_oe', 'push')
_rs_r_oe = scl_get_socket('rs_ctrl_r_oe', 'push')
_rs_oe = _OutputEnable([_rs_p_oe, _rs_r_oe])
_rs_sp_p = scl_get_socket('rs_ctrl_spp_p', 'push')
_rs_sp_r = scl_get_socket('rs_ctrl_spp_r', 'push')


def set_rs(vec):
   _rs_oe.set(1)
   _rp_oe.set(0)
   _rs_sp_p.send(dumps(vec[0]))
   _rs_sp_r.send(dumps(vec[1]))
 
def set_rp(vec):
   _rs_oe.set(1)
   _rp_oe.set(1)
   _hs_oe.set(0)
   _rp_sp_p.send(dumps(vec[0]))
   _rp_sp_r.send(dumps(vec[1]))
 
def set_hs(vec):
   _rs_oe.set(1)
   _rp_oe.set(1)
   _hs_oe.set(1)
   _hp_oe.set(0)
   _hs_sp_n.send(dumps(vec[0]))
   _hs_sp_e.send(dumps(vec[1]))
 
def set_hp(vec):
   _rs_oe.set(1)
   _rp_oe.set(1)
   _hs_oe.set(1)
   _hp_oe.set(1)
   _hp_sp_n.send(dumps(vec[0]))
   _hp_sp_e.send(dumps(vec[1]))
 
