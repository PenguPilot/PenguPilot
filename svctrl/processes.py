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
  
 Process Control Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import subprocess
import os, errno
import signal
import time
from termcolor import red, blue, green
from misc import user_data_dir
import ctypes, ctypes.util as util


def pidfile_from_name(name):
   return user_data_dir + os.sep + 'run' + os.sep + name + '.pid'


def sched_set_prio(pid, prio):
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
   err = libc.sched_setscheduler(pid, _SCHED_FIFO, ctypes.byref(schedParams))
   if err != 0:
      raise OSError('could not set priority, code: %d' % err)


def kill(pid):
   try:
      c = 0
      while True:
         os.kill(pid, signal.SIGTERM)
         time.sleep(1.0)
         c += 1
         if c == 3:
            break
      os.kill(pid, signal.SIGKILL)
   except OSError as err:
      err = str(err)
      if err.find("No such process") == 0:
         raise Exception('could not kill process')


def validate(name):
   pidfile = pidfile_from_name(name)
   try:
      pid = int(file(pidfile).read())
   except:
      pid = None
   try:
      os.kill(pid, 0)
      pid_exists = True
   except OSError as e:
      pid_exists = e.errno == errno.EPERM
   except:
      pid_exists = False
   if pid and not pid_exists:
      os.remove(pidfile)
      return None
   else:
      return pid


def start(name, prio, path, args):
   prio_map = {'sys_high': 99,
               'sys_medium': 93, 
               'sys_low': 92, 
               'app': 91}
   prio = prio_map[prio]
   if args:
      path += ' ' + args
   if validate(name):
      print green('note:') + ' %s is already running' % name
   else:
      print 'starting', blue(name), '...',
      try:
         print path,
         ps = subprocess.Popen(path.split(' '), shell = False, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
         ps.stdout.close()
         ps.stderr.close()
         ps.wait()
         prio_fail = False
         if ps.returncode != 0:
            print red('[ERROR: service quit with code ' + str(ps.returncode) + ']')
         else:
            try:
               pid = None
               for timeout in range(30):
                  time.sleep(0.1)
                  try:
                     pidfile = pidfile_from_name(name)
                     pid = int(file(pidfile).read())
                     break
                  except:
                     pass
               if not pid:
                  raise Exception('no pidfile found')
               sched_set_prio(pid, prio)
            except Exception, e:
               prio_fail = True
            print green('[OK]')
         if prio_fail:
            print red('warning:') + ' could not set process priority'
      except Exception as e:
         print red('[ERROR]: ' + str(e))


def stop(name):
   print 'stopping', blue(name), '...',
   pid = validate(name)
   if pid:
      kill(pid)
      while validate(name):
         time.sleep(1.0)
      print green('[OK]')
   else:
      print red('[ERROR: no such process]')

