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
  
 Process Scheduler Configuration Library

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology
 Inspired by the PsychoPy library Copyright (C) 2009 Jonathan Peirce

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from os import sep
from misc import user_data_dir


def set_priority(name):
   try:
      from yaml import load
      path = user_data_dir() + sep + 'priorities.yaml'
      prio = load(open(path))[name]
      _sched_set_prio(prio)
   except IOError:
      raise ValueError('could not find/read file "%s"; your system is not configured' % path)
   except KeyError:
      raise ValueError('could not retrieve priority of process: %s' % name)


def _sched_set_prio(prio):
   _SCHED_FIFO = 1
   import ctypes, ctypes.util as util
   libc = ctypes.cdll.LoadLibrary(util.find_library('c'))
   if prio > libc.sched_get_priority_max(_SCHED_FIFO):
      raise ValueError('priority too high: %d' % prio)
   elif prio < libc.sched_get_priority_min(_SCHED_FIFO):
      raise ValueError('priority too high: %d' % prio)
   class _SchedParams(ctypes.Structure):
      _fields_ = [('sched_priority', ctypes.c_int)]
   schedParams = _SchedParams()
   schedParams.sched_priority = prio
   err = libc.sched_setscheduler(0, _SCHED_FIFO, ctypes.byref(schedParams))
   if err != 0:
      raise OSError('could not set priority, code: %d' % err)

