from scl import SCL_Reader, scl_get_socket
from time import time, sleep

ts = time()
def cb(data):
   global ts
   if data[0]:
      ts = time()

SCL_Reader('rc', 'sub', callback = cb)
rc_valid = scl_get_socket('rc_valid', 'pub')
state_prev = None
state = 0
while True:
   if time() - ts > 1.0:
      state = 0
   if state_prev != state:
      print state
      rc_valid.send(state)
      state_prev = state
   sleep(0.1)
