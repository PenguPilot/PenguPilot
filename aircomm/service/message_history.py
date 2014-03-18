

from time import time


class MessageHistory:

   def __init__(self, timeout):
      self.timeout = timeout
      self.hist = {}

   def append(self, msg):
      self.hist[msg] = time()

   def check(self, msg):
      if msg in self.hist.keys():
         return False
      self.append(msg)
      rl = []
      ct = time()
      for m, t in self.hist.iteritems():
         if t + self.timeout < ct:
            rl.append(m)
      for m in rl:
         self.hist.pop(m)
      return True

