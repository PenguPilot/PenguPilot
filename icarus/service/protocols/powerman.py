
from power_pb2 import *


class PowerException(Exception):
   pass


class PowerMan:
   
   def __init__(self, ctrl_socket, mon_socket):
      self.ctrl_socket = ctrl_socket
      self.mon_socket = mon_socket

   def _exec(self, cmd):
      req = PowerReq()
      req.cmd = cmd
      self.ctrl_socket.send(req.SerializeToString())
      rep = PowerRep()
      rep.ParseFromString(self.ctrl_socket.recv())
      if rep.status != OK:
         if rep.status == E_SYNTAX:
            print 'received reply garbage'
         else:
            raise PowerException

   def stand_power(self):
      self._exec(STAND_POWER)

   def flight_power(self):
      self._exec(FLIGHT_POWER)

   def read(self):
      state = PowerState()
      data = self.mon_socket.recv()
      state.ParseFromString(data)
      return state

