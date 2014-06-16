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
 
 Movement Activity Class

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from math import hypot
from time import sleep
from util.geomath import LinearInterpolation
from numpy import array, zeros
from pilot_pb2 import *
from activity import Activity, StabMixIn
from util.geomath import gps_add_meters, gps_meters_offset
from util.srtm import SrtmElevMap


_srtm_elev_map = SrtmElevMap()


class MoveActivity(Activity, StabMixIn):


   Z_SPEED_MAX = 2
   SRTM_SAFETY_ALT = 20


   def __init__(self, icarus):
      Activity.__init__(self, icarus)
      self.canceled = False


   def run(self):
      # shortcut identifiers:
      arg = self.icarus.arg
      move_data = arg.move_data
      pilot = self.icarus.pilot
      params = pilot.params
      fsm = self.icarus.fsm
      prev_setp_rel = self.icarus.setpoints
      start_gps = (params.start_lat, params.start_lon)
      prev_setp_gps = gps_add_meters(start_gps, prev_setp_rel[0 : 2])
 
      # calculate target x, y, z and move
      coord = [None, None, None] # x, y, z setpoints
      if arg.glob:
         # set global lat, lon postion:
         glob_sp = [None, None, None]
         for i in xrange(3):
            name = 'p%d' % i
            if move_data.HasField(name):
               glob_sp[i] = getattr(move_data, name)
         print 'p0, p1, p2 = ', glob_sp
         if arg.rel:
            print 'glob, rel'
            # interpret lat, lon, alt as relative
            # covert previous x and y setpoints to rad, using start_lat, start_lon:
            gps = list(prev_setp_gps)
            for i in range(0, 2):
               if glob_sp[i] != None:
                  gps[i] += glob_sp[i]
            # convert from wsg84 to relative:
            coord[0 : 2] = gps_meters_offset(start_gps, gps)
            # add z value:
            coord[2] = prev_setp_rel[2]
            if glob_sp[2] != None:
               coord[2] += glob_sp[2]
         else:
            print 'glob, abs'
            # interpret lat, lon, alt as absolute
            for i in range(0, 2):
               if glob_sp[i] == None:
                  glob_sp[i] = prev_setp_gps[i]
            print start_gps, glob_sp[0 : 2]
            coord[0 : 2] = gps_meters_offset(start_gps, glob_sp[0 : 2])
            if glob_sp[2] != None:
               coord[2] = glob_sp[2] - params.start_alt
            else:
               coord[2] = prev_setp_rel[2]
      else:
         # local position update:
         for i in xrange(3):
            name = 'p%d' % i
            if move_data.HasField(name):
               if arg.rel:
                  print 'local, rel'
                  # relative local coordinate:
                  coord[i] = prev_setp_rel[i] + getattr(move_data, name)
               else:
                  print 'local, abs'
                  # absolute local coordinate:
                  coord[i] = getattr(move_data, name)
            else:
               coord[i] = prev_setp_rel[i]
      
      print 'coord output:', coord
      self.icarus.setpoints = coord
      # set position
      pilot.set_ctrl_param(POS_E, coord[0])
      pilot.set_ctrl_param(POS_N, coord[1])
      
      """
      # did the altitude change?:
      if coord[2] != prev_setp_rel[2]:
         # set up linear z interpolation between start and destination points:
         dist = hypot(prev_setp_rel[0] - coord[0], prev_setp_rel[1] - coord[1])
         z_interp = LinearInterpolation(0.0, start_z, dist, coord[2]) 
         # update z setpoint linearly between starting position and destination:
         target_dist = hypot(pilot.mon[5], pilot.mon[6])
         while target_dist > self.LAT_STAB_EPSILON:
            sleep(1)
            if self.canceled:
               pilot.set_ctrl_param(POS_N, pilot.mon[0])
               pilot.set_ctrl_param(POS_E, pilot.mon[1])
               pilot.set_ctrl_param(POS_U, pilot.mon[2])
               self.stabilize()
               return # not going into hovering state
            z = z_interp(dist - target_dist)
            # check elevation map:
            srtm_z = 1000.0 #_srtm_elev_map.lookup(lat, lon) - params.start_alt
            if z < srtm_alt + self.SRTM_SAFETY_ALT:
               z = srtm_alt + self.SRTM_SAFETY_ALT
            pilot.set_ctrl_param(POS_Z, z)
      """

      self.stabilize()
      if not self.canceled:
         fsm.handle('done')


   def _cancel(self):
      self.canceled = True

