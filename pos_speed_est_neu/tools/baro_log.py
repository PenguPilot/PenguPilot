

from msgpack import loads
from scl import scl_get_socket, SCL_Reader
from world_pos_est import *
from threading import Thread


br = SCL_Reader('baro_raw', 'sub', [0.0, 0.0])
ar = SCL_Reader('acc_world_hp', 'sub', [0.0, 0.0, 0.0])

est_socket = scl_get_socket('world_pos_est', 'sub')
while 1:
    array = loads(est_socket.recv())
    print array[2], array[3], br.data[0], ar.data[2]
