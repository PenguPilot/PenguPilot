from scl import generate_map
from time import sleep

socket = generate_map('aircomm_swarm')['aircomm_out']
while True:
   socket.send('bla')
   sleep(1)
