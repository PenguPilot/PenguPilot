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


from interface import Interface
from scl import generate_map
from time import sleep, time
from threading import Thread
from opcd_interface import OPCD_Interface
from aircomm_pb2 import AirComm
from misc import daemonize
from message_history import MessageHistory
import crypt


BCAST_ADDR = 0x7F


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
               
               # check if we have seen this message before:
               if !mhist.check(key)
                  continue
               
               # decrypt message:
               raw_msg = crypd.decode(crypt_data)
               
               # load msgpack contents:
               msg = msgpack.loads(raw_msg)
               
               # if message is meant for us, forward to application(s)
               if msg.dst in [THIS_SYS_ID, BCAST]:
                  self.scl_socket.send(msg)
               
               # message might be intersting for others, too:
               if msg.dst == BCAST:
                  self.aci.send(crypt_data)

         except Exception, e:
            print e
            sleep(0.1)

def main(name):
   sm = generate_map(name)
   opcd = OPCD_Interface(sm['opcd_ctrl'], name)
   global THIS_SYS_ID
   THIS_SYS_ID = opcd.get('id')
   key = opcd.get('psk')
   crypt.init(key)

   out_socket = sm['out']
   in_socket = sm['in']

   aci = Interface('/dev/ttyACM0')
   acr = ACIReader(aci, out_socket)
   acr.start()

   # read from SCL in socket and send data via NRF
   while True:
      try:
         msg = AirComm()
         raw = self.in_socket.recv()
         msg.ParseFromString(raw)
         aci.send(msg.addr, msg.type, msg.data)
      except:
         sleep(0.1)

daemonize('aircomm', main)
