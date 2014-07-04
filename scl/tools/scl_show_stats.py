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
 the argument is: "/tmp/scl_[socket_name]"

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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
      self.dt = None
      self.socket = socket
      self.name = name
      self.daemon = True
      self.last = time()
      self.filt = None

   def run(self):
      while True:
         try:
            data = self.socket.recv()
            self.dt = time() - self.last
            self.last = time()
         except:
            pass

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
         if thread.dt:
            a = 0.4
            if thread.filt == None:
               thread.filt = 1.0 / thread.dt
            else:
               thread.filt = thread.filt * (1 - a) + (1.0 / thread.dt) * a
            print '%s: %.1f Hz' % (thread.name, thread.filt)
      sleep(1)
   except:
      print 'stopping'
      break

