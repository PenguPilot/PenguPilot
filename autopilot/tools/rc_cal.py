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
  
 Remote Control Calibration Utility

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from time import sleep
from scl import generate_map
from msgpack import loads
from threading import Thread
from copy import copy
from opcd_interface import OPCD_Interface


CHANNELS_MAX = 16

channels_valid = False
channels = [0.0] * CHANNELS_MAX
channels_prev = [0.0] * CHANNELS_MAX
killed = False


class ChannelDetector:

   def __init__(self, spec):
      self.name = spec[0]
      self.max_term, self.min_term = spec[1]
      self.state = 0

   def run(self, channels, channels_prev):
      if self.state == 0:
         print 'please move ' + self.name + ' in any direction'
         self.state = 1
      elif self.state == 1:
         diff_max = 0.0
         for i in range(CHANNELS_MAX):
            diff = abs(channels[i] - channels_prev[i])
            if diff > diff_max:
               i_max = i
               diff_max = diff
         if diff_max > 0.25:
            self.index = i_max
            print 'selected channel', self.index, 'for', self.name, '; please move it', self.max_term
            self.state = 2
      else:
         if self.state == 20:
            self.val_min = channels[self.index]
            print 'min:', self.val_min
         elif self.state == 10:
            self.val_max = channels[self.index]
            print 'max:', self.val_max, 'please move it', self.min_term
         if self.state == 25:
            print 'done for', self.name
            return self.name, self.index, self.val_max, self.val_min
         self.state += 1

   def reset(self):
      self.state = 0


def remote_reader():
   try:
      s = socket_map['remote']
      global channels, channels_valid
      while True:
         data = loads(s.recv())
         channels_valid = bool(data[0])
         channels = data[1:]
   except:
      killed = True

try:
   socket_map = generate_map('rc_cal')
   t = Thread(target = remote_reader)
   t.daemon = True
   t.start()
   sleep(3)
   state = 0
   ud = 'up', 'down'
   rl = 'right', 'left'
   fb = 'forward', 'backward'
   specs = [('gas', ud),
            ('yaw', rl),
            ('pitch', ud),
            ('roll', rl),
            ('two_state', fb),
            ('three_state', fb)]
   detectors = [ ChannelDetector(spec) for spec in specs ]
   channel_map = [ None ] * len(specs)
   states = range(len(specs))
   while not killed:
      if not channels_valid:
         print 'please enable your remote control'
      else:
         result = detectors[state].run(channels, channels_prev)
         if result is not None:
            if result in channel_map:
               print 'channel already used, please try again'
               detectors[state].reset()
            else:
               channel_map[state] = result
               state += 1
         if state == len(specs):
            print channel_map
            break
         channels_prev = copy(channels)
      sleep(0.3)
   if killed:
      raise Exception
   print 'writing to opcd'
   opcd = OPCD_Interface(socket_map['opcd_ctrl'])
   for name, index, max, min in channel_map:
      prefix = 'autopilot.channels.' + name + '.'
      opcd.set(prefix + 'index', index)
      opcd.set(prefix + 'max', max)
      opcd.set(prefix + 'min', min)
   opcd.persist()
   print 'done'
except Exception, e:
   print 'canceled by user', e
