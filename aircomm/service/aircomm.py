#!/usr/bin/env python

from interface import Interface
from scl import generate_map
from time import sleep
from threading import Thread
from opcd_interface import OPCD_Interface
#import msgpack
from misc import daemonize


class ACIReader(Thread):

   def __init__(self, sys_id, aci, scl_socket):
      Thread.__init__(self)
      self.daemon = True
      self.sys_id = sys_id
      self.aci = aci
      self.scl_socket = scl_socket

   def run(self):
      while True:
         try:
            msg = self.aci.receive()
            if True: #msgpack.unpack(msg)[0] == self.sys_id:
               self.aci.send(msg)
               self.scl_socket.send(msg)
         except:
            sleep(1)

def main(name):
   sm = generate_map(name)
   opcd = OPCD_Interface(sm['opcd_ctrl'], name)
   sys_id = opcd.get('id')
   out_socket = sm['out']
   in_socket = sm['in']

   aci = Interface('/dev/ttyACM0')
   acr = ACIReader(sys_id, aci, out_socket)
   acr.start()

   # read from SCL in socket and send data via NRF
   while True:
      raw = in_socket.recv()
      aci.send(raw)


daemonize('aircomm', main)

