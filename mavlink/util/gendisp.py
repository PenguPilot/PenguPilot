
# Generator-based Dispatcher


from Queue import Queue
from threading import Thread, Event
from time import sleep
import re


class RegexDict(dict):


    '''
    regular expression dictionary

    a regular expression is used as the key when retrieving data
    '''

    def __init__(self):
        dict.__init__(self)

    def __setitem__(self, regex, val):
        r = re.compile(regex)
        dict.__setitem__(self, r, val)

    def __getitem__(self, key):
        for regex, val in self.items():
           if regex.match(key):
              return val


class GenDisp(Thread):

   '''
   GenDisp: Generator-based Dispatcher

   The dispatcher is a thread, reading pairs from a
   source until the read function returns None.
   For each pair, the first element (type) defines,
   which generator the second element is passed to.
   When a thread reads from a generator,
   it is woken up when new data delegated to it is available.
   Otherwise, the thread is sleeping.
   
   In comparison to mostly used "handlers",
   the consuming thread can be programmed in a more natural way.
   '''


   def __init__(self, mavio, debug = False):
      Thread.__init__(self)
      self.mavio = mavio
      self.queues = RegexDict()
      self.debug = debug
      self.loss_rate = 0

   def start(self, handlers):
      for handler in handlers:
         handler.start()
      Thread.start(self)

   def generator(self, type):
      if self.debug:
         print 'message reader registered for type', str(type)
      q = Queue()
      self.queues[type] = q
      while True:
         data = q.get()
         if data:
            yield data
            q.task_done()
         else:
            return

   def _read_pair(self):
      message = self.mavio.read()
      return message.get_type(), message

   def run(self):
      loss_hist = [ 0.0 ] * 10
      last_seq = None
      while True:
         try:
            type, data = self._read_pair()
            seq = data.get_seq()
            if seq != 0:
               if last_seq: # out-of-order messages will confuse the calc.
                  loss_hist = loss_hist[1:] + [((seq - last_seq) % 256) - 1]
               sum_lost = float(sum(loss_hist))
               self.loss_rate = sum_lost / (sum_lost + len(loss_hist))
               last_seq = seq
            if self.debug:
               print 'dispatcher is delegating:', str(type)
         except:
            raise
            if self.debug:
               print 'read failed'
            sleep(0.1)
            continue

         try:
            self.queues[type].put(data)
         except:
            if self.debug:
               print 'data', str(data), 'of type', str(type), 'not handled'
 
