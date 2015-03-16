#!/usr/bin/env python
"""
  __________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|

 SCL Live Dump of Msgpack Subscriptions
 the argument is: "~/.PenguPilot/ipc/[socket_name]"

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sys import argv
from msgpack import loads
from zmq import Context, SUB, SUBSCRIBE
from os import listdir
from time import time, sleep
from threading import Thread

files = listdir('/tmp')
context = Context()
threads = []

class Receiver(Thread):

   def __init__(self, socket, name):
      Thread.__init__(self)
      self.cnt = 0
      self.socket = socket
      self.name = name
      self.daemon = True
      self.filt = None

   def run(self):
      while True:
         data = self.socket.recv()
         self.cnt += 1

for file in files:
   try:
      if file[0:4] == 'scl_':
         socket = context.socket(SUB)
         socket.connect("ipc:///tmp/" + file)
         socket.setsockopt(SUBSCRIBE, '')
         thread = Receiver(socket, file[4:])
         thread.start()
         threads.append(thread)
   except:
      pass

print 'preparing...'
sleep(3)
while True:
   try:
      print '---'
      for thread in threads:
         if thread.cnt:
            cnt = thread.cnt
            thread.cnt = 0
            a = 0.5
            if thread.filt == None:
               thread.filt = float(cnt) / 5
            else:
               thread.filt = thread.filt * (1 - a) + float(cnt) / 5 * a
            print '%s: %.1f Hz' % (thread.name, thread.filt)
      sleep(5)
   except:
      print 'stopping'
      break

