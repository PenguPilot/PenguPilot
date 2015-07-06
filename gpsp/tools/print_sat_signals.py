from scl import scl_get_socket
from msgpack import loads

s = scl_get_socket('sats', 'sub')
while True:
   d = loads(s.recv())
   sum = 0.0
   for x in d:
      sum += x[4]
      print x[4],
   sum /= len(d)
   print '| avg: %.1f' % sum
