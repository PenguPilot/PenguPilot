#!/bin/env python

from gps_data_pb2 import GpsData
from msgpack import loads, dumps
from threading import Thread
from scl import generate_map
from misc import daemonize


class GPS_Reader(Thread):
   
   def __init__(self, socket):
      Thread.__init__(self)
      self.daemon = True
      self.socket = socket

   def run(self):
      i = 0
      while True:
         data = self.socket.recv()
         if i == 5:
            i = 0
            gps = GpsData()
            gps.ParseFromString(data)
            self.data = gps
         i += 1


def main(name):
   map = generate_map(name)
   gps_reader = GPS_Reader(map['gps'])
   gps_reader.start()
   wifi_socket = map['networks']
   f = file("/tmp/wifi.log", "w")
   while True:
      try:
         measure = loads(wifi_socket.recv())
         gps = gps_reader.data
         f.write('%f %f %s %d\n' % (gps.lat, gps.lon, measure[0], measure[1]))
         f.flush()
      except:
         pass

daemonize('wifi_loc', main)

