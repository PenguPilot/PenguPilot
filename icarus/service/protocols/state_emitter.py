
# ICARUS state emitter
# 
# Author: Tobias Simon, Ilmenau University of Technology
#
# Purpose:
#  - translates flight_sm states to icarus StateUpdate messages
#  - sends icarus StateUpdate messages using the given socket
#


from icarus_pb2 import StateUpdate
from util.flight_state import to_protocol


class StateEmitter:

   def __init__(self, socket):
      self._socket = socket

   def send(self, state):
      sm = StateUpdate()
      sm.state = to_protocol(state)
      self._socket.send(sm.SerializeToString())

