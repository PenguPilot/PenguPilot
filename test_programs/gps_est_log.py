from scl import scl_get_socket, SCL_Reader
from msgpack import loads

gps = SCL_Reader('gps_rel', 'sub', [0.0, 0.0, 0.0, 0.0])
est = scl_get_socket('pos_speed_est_neu', 'sub')
while True:
   e = loads(est.recv())
   r = gps.data
   print e[4], e[5], e[6], e[7], r[0], r[1], r[2], r[3]
