
'''
mavlink IO functions

Copyright Tobias Simon 2012
Released under GNU GPL version 3 or later
'''

import pymavlink.mavlinkv10 as mavlink
import serial
import time
import random
import socket


class MAVIO:
    
   def __init__(self, source_system):
      self.mav = mavlink.MAVLink(self, srcSystem = source_system)
      self.mav.robust_parsing = True

   def read(self):
      while True:
         data = self._read_bytes()
         for byte in data:
            msg = self.mav.parse_char(byte)
            if msg is not None:
               return msg


class MAVIO_Serial(MAVIO):
    
   def __init__(self, device, baud, source_system):
      MAVIO.__init__(self, source_system)
      self.device = device
      self.baud = baud
      self.port = serial.Serial(self.device, self.baud,
         timeout = None, bytesize = serial.EIGHTBITS,
         parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE,
         dsrdtr = False, rtscts = False, xonxoff = False,
         writeTimeout = None)
      self.port.setDTR(False)

   def _read_bytes(self):
      return self.port.read(1024)

   def write(self, data):
      self.port.write(data)


class MAVIO_UDP(MAVIO):
    
   def __init__(self, address, port, source_system):
      MAVIO.__init__(self, source_system)
      self.address = address
      self.port = port
      self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


   def _read_bytes(self):
      return self.socket.recvfrom(1024)

   def write(self, data):
      self.socket.sendto(data, (self.address, self.port))

