from scl import generate_map
from sys import argv

assert len(argv) == 3
socket = generate_map(argv[1])[argv[2]]
while True:
   print len(socket.recv())
