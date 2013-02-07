

from math import pi
from hardware.hardware import *
from standard.standard import *
from algorithms.geometry import *
from control_modes.control_modes import *

from platforms.arcade import *
from platforms.standard import *
from emitter.emitter import *

class Settings(StandardSettings):

   def __init__(self):
      StandardSettings.__init__(self)
      
      # select and configure platform:
      cap = ['ultra', 'gps', 'baro', 'voltage', 'current']
      self.platform = ARCADE_Quad('holger', cap)
      self.battery = Battery(4, 6.6, 3.3)


conf = Settings()
from walk import walk
walk(conf)

