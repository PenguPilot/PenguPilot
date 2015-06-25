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
  
 Scheduler Interface for current Process

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import os
import ctypes, ctypes.util as util


def sched_set_prio(prio):
   _SCHED_FIFO = 1
   libc = ctypes.cdll.LoadLibrary(util.find_library('c'))
   if prio > libc.sched_get_priority_max(_SCHED_FIFO):
      raise ValueError('priority too high: %d' % prio)
   elif prio < libc.sched_get_priority_min(_SCHED_FIFO):
      raise ValueError('priority too high: %d' % prio)
   class _SchedParams(ctypes.Structure):
      _fields_ = [('sched_priority', ctypes.c_int)]
   schedParams = _SchedParams()
   schedParams.sched_priority = prio
   libc.sched_setscheduler(os.getpid(), _SCHED_FIFO, ctypes.byref(schedParams))

