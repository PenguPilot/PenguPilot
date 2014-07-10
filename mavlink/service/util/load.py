
from psutil import cpu_percent
from time import sleep
from threading import Thread


class LoadReader(Thread):

   def __init__(self):
      Thread.__init__(self)
      self.daemon = True
      self.load_avg = [ float(cpu_percent()) ] * 10
      self.load = 0

   def run(self):
      while True:
         self.load_raw = float(cpu_percent())
         self.load_avg = self.load_avg[1 : ] + [ self.load_raw ]
         load = 0.0
         for entry in self.load_avg:
            load += entry
         sleep(1.0)
         self.load = load / len(self.load_avg)

