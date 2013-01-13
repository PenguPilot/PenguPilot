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
  
 Miscellaneous Code

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
 Partially taken from PsychoPy library Copyright (C) 2009 Jonathan Peirce

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sys import exit
from signal import pause
from daemon import DaemonContext
try:
   from daemon import pidlockfile
except:
   from daemon import pidfile as pidlockfile
from threading import Thread
from os import getenv, sep
import errno
from time import time
import ctypes, ctypes.util


# user data directory:

def user_data_dir():
   return getenv('HOME') + sep + '.PenguPilot'


# Hysteresis class:

class Hysteresis:

   def __init__(self, timeout):
      self.timeout = timeout
      self.start_time = None

   def set(self):
      if self.start_time == None:
         self.start_time = time()
      elif self.start_time + self.timeout < time():
         return True
      return False
   
   def reset(self):
      self.start_time = None



# process and thread daemonization:

def _main_wrapper(name, main):
   main(name)
   pause()


def daemonize(name, main):
   try:
      run_dir = user_data_dir() + sep + 'run'
      pidf = pidlockfile.PIDLockFile(run_dir + sep + name + '.pid')
      pidf.acquire(timeout = 1.0)
      pidf.release()
      with DaemonContext(pidfile = pidf):
         _main_wrapper(name, main)
   except Exception as e:
      print 'Could not daemonize:', str(e)
      exit(1)


def start_daemon_thread(target):
   thread = Thread(target = target)
   thread.daemon = True
   thread.start()
   return thread


def await_signal():
   try:
      pause()
   except:
      print 'killed by user'




# process priority modification:

_c = ctypes.cdll.LoadLibrary(ctypes.util.find_library('c'))

_SCHED_FIFO = 1

class _SchedParams(ctypes.Structure):
   _fields_ = [('sched_priority', ctypes.c_int)]


def sched_get_minprio():
   return _c.sched_get_priority_min(_SCHED_FIFO)


def sched_get_maxprio():
   return _c.sched_get_priority_max(_SCHED_FIFO)


def sched_rtprio(priority):
    priority = int(priority)
    if priority > sched_get_maxprio():
       raise ValueError('priority too high')
    elif priority < sched_get_minprio():
       raise ValueError('priority too high')
    schedParams = _SchedParams()
    schedParams.sched_priority = priority
    err = _c.sched_setscheduler(0, _SCHED_FIFO, ctypes.byref(schedParams))
    if err != 0:
      raise OSError('could not set priority, code: %d' % err)

