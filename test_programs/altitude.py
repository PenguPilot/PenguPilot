
from scl import SCL_Reader, scl_get_socket
from gps_msgpack import ALT
from time import sleep
from msgpack import loads

gps = SCL_Reader('gps', 'sub')
elev = SCL_Reader('baro_elev', 'sub')
pos_est_socket = scl_get_socket('pos_speed_est_neu', 'sub')
sleep(3)
while True:
   pe = loads(pos_est_socket.recv())
   print gps.data[ALT], elev.data[0], pe[2]
