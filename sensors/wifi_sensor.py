#!/usr/bin/env python

# WiFi network scanner using wlan1


from networks_pb2 import Measurement
from scl import generate_map
import time
from misc import daemonize
from subprocess import call, Popen, PIPE


def main(name):
   dev = 'wlan1'
   call(['ifconfig', dev, 'down'])
   call(['iwconfig', dev, 'mode', 'monitor'])
   call(['iwconfig', dev, 'channel', '1'])
   call(['ifconfig', dev, 'up'])
   socket = generate_map(name)['networks']
   p = Popen(['tcpdump', '-l', '-e', '-i', dev], stdout = PIPE, stderr = PIPE)
   while True:
      line = p.stdout.readline()
      list = line.split(' ')
      if len(list) == 46:
         measure = Measurement()
         measure.mac = list[11][3:]
         measure.rssi = int(list[6][0:-2])
         print measure
         socket.send(measure.SerializeToString())


main('wifi_sensor')
daemonize('wifi_sensor', main)

