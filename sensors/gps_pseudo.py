#!/usr/bin/python
# pseudo GPS sensor for testing and demonstration

from random import random
from zmq import HWM
from time import sleep
from misc import daemonize
from gps_data_pb2 import GpsData, SatInfo
from scl import generate_map


def main(name):
   socket = generate_map(name)['gps']
   socket.setsockopt(HWM, 1)
   while True:
      print 'send'
      gps_data = GpsData()
      sleep(0.2)
      gps_data.fix = 3
      gps_data.time = 'NONE'
      gps_data.lat = 50.0
      gps_data.lon = 10.0
      gps_data.alt = 500.0
      gps_data.sats = 6
      gps_data.hdop = 1.0
      gps_data.vdop = 2.0
      for i in range(0, 5):
         si = gps_data.satinfo.add()
         si.id = i
         si.in_use = int(random() + 0.5)
         si.elv = int(random() * 100) - 50
         si.azimuth = int(random() * 100) - 50
         si.sig = int(random() * 100)
      socket.send(gps_data.SerializeToString())

main('gps_sensor')
#daemonize('gps_sensor', main)

