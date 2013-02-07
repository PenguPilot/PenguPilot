

from math import pi
from settings.hardware import *
from settings.standard import *
from tools.geometry import *
from settings.ctrl_modes import *

from settings.arcade import *
from tools.emitter import *

class Settings(StandardSettings):

   def __init__(self):
      StandardSettings.__init__(self)
      
      # select and configure platform:
      cap = ['ultra', 'gps', 'baro', 'voltage', 'current']
      self.platform = ARCADE_Quad('holger', cap)
      self.battery = Battery(4, 6.6, 3.3)


conf = Settings()
from tools.emitter import walk
walk(conf)

