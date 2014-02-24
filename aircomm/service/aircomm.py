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
  
 Aircomm Service

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from base64 import b64encode
from interface import Interface
from scl import generate_map
from time import sleep, time
from threading import Thread
from opcd_interface import OPCD_Interface
from msgpack import loads, dumps
from misc import daemonize
from message_history import MessageHistory
import crypt
from aircomm_shared import BCAST


class ACIReader(Thread):

   def __init__(self, aci, scl_socket):
      Thread.__init__(self)
      self.daemon = True
      self.aci = aci
      self.scl_socket = scl_socket
      self.mhist = MessageHistory(60)

   def run(self):
      s = 0
      while True:
         try:
            # receive encrypted message:
            crypt_data = self.aci.receive()
            if crypt_data:
               print 'received encrypted data:', b64encode(crypt_data), 'orig len: %d' % len(crypt_data)
               
               # check if we have seen this message before:
               if not self.mhist.check(crypt_data):
                  print 'message already seen; skipping'
                  continue
               
               # decrypt message:
               raw_msg = crypt.decrypt(crypt_data)
               
               # load msgpack contents:
               try:
                  msg = loads(raw_msg)
               except:
                  print 'could not unpack data'
                  continue
               
               # if message is meant for us, forward to application(s)
               if msg[0] in [THIS_SYS_ID, BCAST]:
                  msg_tail = msg[1:]
                  print 'message for me:', msg_tail
                  #self.scl_socket.send(dumps(msg_tail))

               if msg[0] != THIS_SYS_ID:
                  print 'broadcasting message'
                  self.aci.send(crypt_data)

         except Exception, e:
            print e
            sleep(1)


def main(name):
   sm = generate_map(name)
   opcd = OPCD_Interface(sm['opcd_ctrl'], name)
   global THIS_SYS_ID
   THIS_SYS_ID = opcd.get('id')
   key = opcd.get('psk')
   crypt.init(key)

   out_socket = None#sm['out']
   in_socket = sm['in']

   aci = Interface('/dev/ttyACM0')
   acr = ACIReader(aci, out_socket)
   acr.start()

   # read from SCL in socket and send data via NRF
   while True:
      data = loads(in_socket.recv())
      if len(data) == 2:
         msg = [data[0], THIS_SYS_ID, data[1]]
      elif len(data) > 2:
         msg = [data[0], THIS_SYS_ID] + data[1:]
      else:
         continue
      crypt_msg = crypt.encrypt(dumps(msg))
      aci.send(crypt_msg)

daemonize('aircomm', main)

