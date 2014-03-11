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
 
 ICARUS
 - takes care that the UAV system is always in a valid state by using the flight state machine
 - accepts or rejects user commands
 - performs emergency landing if battery voltage is too low
 - performs flight distance estimation based on speed and estimated flight time
 - learns possible landing spots and navigates there if the battery goes low
 - publishes state updates via SCL

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from core_pb2 import *
from math import atan2
from time import sleep, time
from icarus_pb2 import TAKEOFF, LAND, MOVE, STOP, ROT
from interface.icarus_server import ICARUS_Server, ICARUS_Exception
from activities.takeoff import TakeoffActivity
from activities.land import LandActivity
from activities.move import MoveActivity
from activities.stop import StopActivity
from activities.dummy import DummyActivity
from flight_sm import flight_sm, flight_Hovering
from flight_sm import flight_Standing, flight_Moving, flight_Stopping
from util.geomath import bearing, gps_add_meters
from misc import *
from misc import daemonize
from scl import generate_map
from core_interface import CoreInterface
from interface.state_emitter import StateEmitter
from powerman import PowerMan
from logging import basicConfig as log_config, debug as log_debug
from logging import info as log_info, warning as log_warn, error as log_err
from logging import DEBUG
from os import sep
from misc import user_data_dir
from util.landing_spots import LandingSpots
from icarus_interface import ICARUS_MissionFactory
from util.flight_state import to_string


class ICARUS:

   def __init__(self, sockets):
      logfile = user_data_dir() + sep + 'ICARUS.log'
      log_config(filename = logfile, level = DEBUG,
                 format = '%(asctime)s - %(levelname)s: %(message)s')
      log_info('icarus starting up')
      self.setpoints = [0.0, 0.0, 2.0] # x, y, z
      self.flight_time = 0
      self.icarus_takeover = False
      self.emergency_land = False
      self.factory = ICARUS_MissionFactory
      self.fsm = flight_sm(self)
      self.landing_spots = LandingSpots(3.0)
      self.core = CoreInterface(sockets['core'], sockets['mon'])
      #self.gps_shifter = GPS_Shifter()
      self.state_emitter = StateEmitter(sockets['state'])
      self.powerman = PowerMan(sockets['power_ctrl'], sockets['power_mon'])
      start_daemon_thread(self.power_state_monitor)
      start_daemon_thread(self.state_time_monitor)
      start_daemon_thread(self.core_monitor)
      self.activity = DummyActivity()
      self.activity.start()
      self.icarus_srv = ICARUS_Server(sockets['ctrl'], self)
      self.icarus_srv.start()
      log_info('icarus up and running')


   def state_time_monitor(self):
      log_info('starting state time monitor')
      while True:
         if self.fsm._state != flight_Standing:
            # count the time for "in-the-air-states":
            self.flight_time += 1
         else:
            # use system uptime here:
            up_file = open("/proc/uptime", "r")
            up_list = up_file.read().split()
            self.uptime = int(up_list[0])
            up_file.close()
         sleep(1)


   def core_monitor(self):
      log_info('starting core state monitor')
      rc_timeout = 1.0
      return_when_signal_lost = False
      self.mon_data = MonData()
      last_valid = time()
      while True:
         self.core.mon_read(self.mon_data)
         self.kalman_lat, self.kalman_lon = gps_add_meters((self.core.params.start_lat,
                                                           self.core.params.start_lon),
                                                           self.setpoints[0 : 2])
         if self.mon_data.signal_valid:
            last_valid = time()
         else:
            if time() - rc_timeout < last_valid and return_when_signal_lost:
               if not self.icarus_takeover:
                  self.icarus_takeover = True
                  log_err('invalid RC signal, disabling mission interface')


   def power_state_monitor(self):
      log_info('starting power state monitor')
      while True:
         self.power_state = self.powerman.read()
         if self.power_state.critical:
            log_warn('critical power state: emergency landing')
            # disable system interface and take over control:
            self.icarus_takeover = True
            if not self.emergency_land:
               self.emergency_landing()


   def move_and_land(self, x, y):
      # try to stop and move somewhere
      try:
         self.fsm.stop()
      except:
         pass
      try:
         self.arg = IcarusReq()
         self.arg.type = MOVE
         self.arg.move_data.p0 = x
         self.arg.move_data.p1 = y
         self.fsm.move()
      except:
         pass
      try:
         self.fsm.land()
      except:
         pass


   def emergency_landing(self):
      return # TODO: test
      # we don't care about the system's state:
      # just try to stop and land it!
      try:
         self.fsm.stop()
      except:
         pass
      try:
         self.fsm.land()
      except:
         pass
      # after emergency landing, the interface will stay locked
      # and power circruitry will go offline



   def rotate(self, arg):
      if len(self.arg.pos) == 1:
         self.yaw_target = self.arg.pos[0]
      else:
         self.yaw_target = self.arg.pos[0], self.arg.pos[1]


   def yaw_update_thread(self):
      '''
      This method/thread is responsible for updating the yaw setpoint of the system.
      The code is only executed when the system is in a valid state.
      '''
      prev_yaw = None
      min_rot_z_ground = 1.0
      while True:
         sleep(1)
         # when landing: setting a new rotation setpoint is not allowed:
         if self.fsm._state is flight_Landing:
            continue
         if self.mon_data.z_ground < self.min_rot_z_ground:
            print 'system is able to rotate'
            try:
               if isinstance(self.yaw_target, float):
                  print 'fixed yaw mode'
                  yaw = self.yaw
               else:
                  poi_x = self.yaw_target[0]
                  poi_y = self.yaw_target[1]
                  print 'POI mode, x =', poi_x, ' y =', poi_y
                  yaw = atan2(self.mon_data.y - poi_y, self.mon_data.x - poi_x)
               if prev_yaw != yaw:
                  print 'setting yaw to:', yaw
                  self.core.set_ctrl_param(POS_YAW, yaw)
               prev_yaw = yaw
            except AttributeError:
               pass


   # called by ICARUS command interface:
   def handle(self, req, local = False):
      if not local and self.icarus_takeover:
         msg = 'request rejected due to emergency take-over'
         log_err(msg)
         raise ICARUS_Exception(msg)
      self.arg = req
      if req.type == TAKEOFF:
         self.fsm.takeoff()
      elif req.type == LAND:
         self.fsm.land()
      elif req.type == MOVE:
         self.fsm.move()
      elif req.type == STOP:
         self.fsm.stop()
      elif req.type == ROT:
         self.rotate(arg)


   # following _prefix methods are called internally from state machine
   # and must not be called explicitly

   def _error(self):
      msg = 'flight state machine error'
      log_err(msg)
      raise ICARUS_Exception(msg)


   def _broadcast(self):
      log_info('new state: %s' % to_string(self.fsm._state))
      self.state_emitter.send(self.fsm._state)


   def _save_power_activity(self):
      log_info('standing')
      self.powerman.stand_power()


   def _takeoff_activity(self):
      log_info('taking off')
      self.landing_spots.add((self.mon_data.x, self.mon_data.y))
      self.activity.cancel_and_join()
      self.powerman.flight_power()
      self.yaw_setpoint = self.mon_data.yaw
      self.activity = TakeoffActivity(self.fsm, self)
      self.activity.start()


   def _land_activity(self):
      log_info('landing')
      self.activity.cancel_and_join()
      self.activity = LandActivity(self)
      self.activity.start()


   def _move_activity(self):
      log_info('moving')
      self.activity.cancel_and_join()
      self.activity = MoveActivity(self)
      self.activity.start()


   def _stop_activity(self):
      log_info('stopping')
      self.activity.cancel_and_join()
      self.activity = StopActivity(self)
      self.activity.start()



def main(name):
   sockets = generate_map(name)
   ICARUS(sockets)
   await_signal()


main('icarus')
#daemonize('icarus', main)

