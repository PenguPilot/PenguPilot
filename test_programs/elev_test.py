from scl import scl_get_socket
from msgpack import dumps
from time import sleep

s = scl_get_socket('vp_ctrl_sp', 'pub')
sleep(1)
s.send(dumps(['elev', 1.0]))
