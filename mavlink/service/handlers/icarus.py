
from icarus_interface import ICARUS_Error
from threading import Thread
from mavlinkv10 import *


'''
MAV_CMD_ACK_OK
MAV_CMD_ACK_ERR_FAIL
'''

class MissionHandler(Thread):

   def __init__(self, dispatcher, icarus_interface):
      Thread.__init__(self)
      self.dispatcher = dispatcher
      self.icarus_interface = icarus_interface


   def _send_ack(self, status):
      self.dispatcher.mavio.mav.command_ack_send(self._command, status)


   def _takeoff(self, arg):
      try:
         self.icarus_interface.takeoff()
         self._send_ack(MAV_CMD_ACK_OK)
      except icarusError:
         self._send_ack(MAV_CMD_ACK_ERR_FAIL)


   def _land(self, arg):
      try:
         self.icarus_interface.land()
         self._send_ack(MAV_CMD_ACK_OK)
      except:
         self._send_ack(MAV_CMD_ACK_ERR_FAIL)

   def run(self):
      cmd_map = {MAV_CMD_NAV_TAKEOFF: self._takeoff,
                 MAV_CMD_NAV_LAND: self._land}
      for e in self.dispatcher.generator('COMMAND_LONG'):
         self._command = e.command
         try:
            method = cmd_map[e.command]
         except KeyError:
            pass #print 'command not handled:', e

