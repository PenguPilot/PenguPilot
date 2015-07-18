from scl import scl_get_socket
from geomath import rad2deg

s = scl_get_socket('orientation', 'sub')
while True:
   y, p, r = map(rad2deg, s.recv())
   print 'Euler (y, p, r) =', y, p, r

