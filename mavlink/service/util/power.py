
from power_pb2 import *
from threading import Thread
from time import sleep


class PowerReader(Thread):

   def __init__(self, socket):
      Thread.__init__(self)
      self.daemon = True
      self.socket = socket
      self.power = PowerState()
      self.init_done = False

   def wait_data(self):
      while not self.init_done:
         sleep(1)

   def run(self):
      while True:
         data = self.socket.recv()
         self.power.ParseFromString(data)
         self.init_done = True

