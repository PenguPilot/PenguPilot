
from core_pb2 import *


class CoreError(Exception):

   def __init__(self, status, err_msg):
      self.status = status
      self.err_msg = err_msg

   def __repr__(self):
      err_map = {E_SYNTAX: 'E_SYNTAX', E_SEMANTIC: 'E_SEMANTIC', E_HARDWARE: 'E_HARDWARE'}
      return 'class: ' + err_map[self.status] + ' message: ' + self.err_msg


class CoreInterface:

   def __init__(self, ctrl_socket, mon_socket):
      self.ctrl_socket = ctrl_socket
      self.params = self.get_params()
      self.mon_socket = mon_socket

   def _exec(self, req):
      self.ctrl_socket.send(req.SerializeToString())
      rep = CoreRep()
      rep.ParseFromString(self.ctrl_socket.recv())
      if rep.status != 0:
         raise CoreError(rep.status, rep.err_msg)
      return rep

   def mode_normal(self):
      req = CoreReq()
      req.type = MODE_NORMAL
      self._exec(req)

   def mode_cal(self):
      req = CoreReq()
      req.type = MODE_CAL
      self._exec(req)

   def spin_up(self):
      req = CoreReq()
      req.type = SPIN_UP
      self._exec(req)

   def spin_down(self):
      req = CoreReq()
      req.type = SPIN_DOWN
      self._exec(req)

   def reset_ctrl(self):
      req = CoreReq()
      req.type = RESET_CTRL
      self._exec(req)
  
   def set_ctrl_param(self, param, val):
      req = CoreReq()
      req.type = SET_CTRL_PARAM
      req.ctrl_data.param = param
      req.ctrl_data.val = val
      self._exec(req)

   def get_params(self):
      req = CoreReq()
      req.type = GET_PARAMS
      rep = self._exec(req)
      return rep.params

   def mon_read(self, mon):
      data = self.mon_socket.recv()
      mon.ParseFromString(data)

