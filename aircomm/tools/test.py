from scl import generate_map
from time import sleep
from msgpack import dumps

socket = generate_map('aircomm_swarm')['aircomm_out']
while True:
   socket.send(dumps([10, 'test']))
   sleep(1)
