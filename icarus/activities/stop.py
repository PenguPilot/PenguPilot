
from core_pb2 import *
from activity import Activity, StabMixIn


class StopActivity(Activity, StabMixIn):

   def __init__(self, fsm, core, mon_data):
      Activity.__init__(self)
      self.fsm = fsm
      self.core = core
      self.mon_data = mon_data
      self.canceled = False

   def run(self):
      core = self.core
      mon_data = self.mon_data
      self.core.set_ctrl_param(POS_X, mon_data.x)
      self.core.set_ctrl_param(POS_Y, mon_data.y)
      self.core.set_ctrl_param(POS_Z, mon_data.z)
      self.stabilize()
      self.fsm.done()

