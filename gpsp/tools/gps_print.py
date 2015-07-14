from scl import scl_get_socket
from msgpack import loads
from gps_msgpack import *


s = scl_get_socket('gps', 'sub')
while True:
   gps_data = loads(s.recv())
   f = fix(gps_data)
   if f >= 2:
      print gps_data[LAT], gps_data[LON]
