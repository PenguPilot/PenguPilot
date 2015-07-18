from scl import scl_get_socket

s = scl_get_socket('sats', 'sub')
while True:
   sum = 0.0
   for x in s.recv():
      sum += x[4]
      print x[4],
   sum /= len(d)
   print '| avg: %.1f' % sum
