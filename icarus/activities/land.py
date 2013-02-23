
from time import sleep
from core_pb2 import *

from activity import Activity


class LandActivity(Activity):

   MIN_HOVERING_ALT = 0.57

   def __init__(self, icarus):
      Activity.__init__(self, icarus)

   def run(self):
      core = self.icarus.core
      mon_data = self.icarus.mon_data
      fsm = self.icarus.fsm
      
      core.set_ctrl_param(POS_Z_GROUND, self.MIN_HOVERING_ALT / 3.0)
      while mon_data.z_ground > self.MIN_HOVERING_ALT:
         sleep(self.POLLING_TIMEOUT)
      core.spin_down()
      fsm.landing_done()

