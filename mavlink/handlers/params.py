
from opcd_interface import OPCD_Interface
from threading import Thread
import re
from scl import generate_map
#from mavlinkv10 import MAVLINK_TYPE_FLOAT_T, MAVLINK_TYPE_INT32_T

MAVLINK_TYPE_FLOAT_T = 1
MAVLINK_TYPE_INT32_T = 2

class ParamHandler(Thread):

   def __init__(self, dispatcher):
      Thread.__init__(self)
      self.dispatcher = dispatcher
      self.opcd_interface = OPCD_Interface(generate_map('mavlink')['ctrl'])
      self.param_map = {}
      self.param_name_map = {}
      list = self.opcd_interface.get('')
      c = 0
      type_map = {float: MAVLINK_TYPE_FLOAT_T, long: MAVLINK_TYPE_INT32_T}
      cast_map = {float: float, long: int}
      for name, val in list:
         try:
            type = type_map[val.__class__]
            self.param_map[c] = name, type, cast_map[val.__class__]
            self.param_name_map[c] = c, type, cast_map[val.__class__]
            c += 1
         except Exception, e:
            print str(e)
   
   def run(self):
      for e in self.dispatcher.generator('PARAM_'):
         print e
         if e.get_type() == 'PARAM_REQUEST_LIST':
            list = self.opcd_interface.get('')
            for index, (name, type, cast) in self.param_map.items():
               try:
                  val = self.opcd_interface.get(name)
                  name_short = re.sub('(?P<foo>\w)\w*\.', '\g<foo>.', name)
                  name_short = re.sub('_', '-', name)
                  name_short = re.sub('\.', '_', name_short)
                  self.dispatcher.mavio.mav.param_value_send(name_short, float(val), type, len(self.param_map), index)
               except Exception, ex:
                  print str(ex)

         elif e.get_type() == 'PARAM_REQUEST_READ':
            index = e.param_index
            try:
               if index == -1:
                  try:
                     name = e.param_id
                     index, type, case = self.param_name_map[name]
                  except KeyError:
                     print 'unkwnown param requested:', name
                     raise
               else:
                  try:
                     name, type, cast = self.param_map[index]
                  except KeyError:
                     print 'unkwnown param requested:', index
                     raise
            except KeyError:
               continue

            try:
               val = self.opcd_interface.get(name)
               name_short = re.sub('(?P<foo>\w)\w*\.', '\g<foo>.', name)
               name_short = re.sub('_', '-', name)
               name_short = re.sub('\.', '_', name_short)
               self.dispatcher.mavio.mav.param_value_send(name_short, float(val), type, len(self.param_map), index)
            except Exception, ex:
               raise
               print str(ex)

