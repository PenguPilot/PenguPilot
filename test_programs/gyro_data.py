
from msgpack import dumps
from scl import scl_get_socket
from random import random
from time import sleep

s = scl_get_socket('gyro_raw', 'pub')
while True:
   s.send(dumps([random() - 0.3, random() - 0.0, random() + 0.5]))
   sleep(0.005)

