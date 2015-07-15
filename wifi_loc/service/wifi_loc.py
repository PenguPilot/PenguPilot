#!/bin/env python

from threading import Thread
from scl import generate_map
from misc import daemonize, RateTimer
from gps_msgpack import *


class GPS_Reader(Thread):
   
   def __init__(self, socket):
      Thread.__init__(self)
      self.daemon = True
      self.socket = socket

   def run(self):
      rt = RateTimer(1)
      while True:
         self.data = self.socket.recv()


def main(name):
   map = generate_map(name)
   gps_reader = GPS_Reader(map['gps'])
   gps_reader.start()
   wifi_socket = map['networks']
   f = file("/tmp/wifi.log", "w")
   while True:
      try:
         measure = wifi_socket.recv()
         gps = gps_reader.data
         f.write('%f %f %s %d\n' % (gps[LAT], gps[LON], measure[0], measure[1]))
         f.flush()
      except:
         pass

daemonize('wifi_loc', main)

