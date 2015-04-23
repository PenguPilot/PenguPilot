
from msgpack import dumps
from scl import scl_get_socket
from time import sleep
from sys import argv

try:
    s = scl_get_socket('inten', 'pub')
    while True:
        s.send(dumps(int(argv[1])))
        sleep(0.1)
except:
   print 'usage: inten_test.py value'
