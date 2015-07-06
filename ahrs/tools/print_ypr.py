from scl import scl_get_socket
from msgpack import loads
from geomath import rad2deg

s = scl_get_socket('orientation', 'sub')
while True:
   y, p, r = map(rad2deg, loads(s.recv())[0:3])
   print 'Euler (y, p, r) =', y, p, r

