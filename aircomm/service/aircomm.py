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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from base64 import b64encode
from interface import NRF_Interface, ZMQ_Interface
from scl import generate_map
from time import sleep, time
from threading import Thread
from opcd_interface import OPCD_Interface
from msgpack import loads, dumps
from misc import daemonize
from message_history import MessageHistory
import crypt
from aircomm_shared import BCAST, BCAST_NOFW


class ACIReader(Thread):

   def __init__(self, aci, scl_socket, mhist):
      Thread.__init__(self)
      self.daemon = True
      self.aci = aci
      self.scl_socket = scl_socket
      self.mhist = mhist

   def run(self):
      s = 0
      while True:
         try:
            # receive encrypted message:
            crypt_data = self.aci.receive()
            if crypt_data:
               
               # check if we have seen this message before:
               if not self.mhist.check(crypt_data):
                  continue
               
               # decrypt message:
               raw_msg = crypt.decrypt(crypt_data)
               
               # load msgpack contents:
               try:
                  msg = loads(raw_msg)
               except Exception, e:
                  continue
               
               addr = msg[0]
               # if message is meant for us, forward to application(s):
               if addr in [THIS_SYS_ID, BCAST, BCAST_NOFW]:
                  msg_scl = dumps(msg[1:]) # strip the type and pack again
                  self.scl_socket.send(msg_scl)

               # if the message (a) is not meant for us or (b) is NOFW, re-broadcast:
               if addr not in [THIS_SYS_ID, BCAST_NOFW]:
                  self.aci.send(crypt_data)

         except Exception, e:
            sleep(1)


def main(name):
   sm = generate_map(name)
   
   opcd = OPCD_Interface(sm['opcd_ctrl'])
   platform = opcd.get('platform')
   device = opcd.get(platform + '.nrf_serial')
   
   global THIS_SYS_ID
   THIS_SYS_ID = opcd.get('aircomm.id')
   key = opcd.get('aircomm.psk')
   crypt.init(key)
   mhist = MessageHistory(60)

   out_socket = sm['aircomm_out']
   in_socket = sm['aircomm_in']

   aci = ZMQ_Interface()
   acr = ACIReader(aci, out_socket, mhist)
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
      crypt_data = crypt.encrypt(dumps(msg))
      mhist.append(crypt_data)
      aci.send(crypt_data)


daemonize('aircomm', main)

