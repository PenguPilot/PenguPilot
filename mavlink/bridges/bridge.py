
class Bridge:

   '''
   A bridge connects a data source to a data sink (usually via SCL)
   and implements the required data conversion in order to send it to
   the MAVLink peer.
   '''

   def __init__(self, socket_map, mav_iface, send_interval):
      self.socket_map = socket_map
      self.mav_iface = mav_iface
      self.send_interval = send_interval

