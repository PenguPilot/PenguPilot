import curses
import curses.wrapper
from random import random
import numpy as np
from scl import scl_get_socket, SCL_Reader
from msgpack import loads
from time import sleep

scr = curses.initscr()
h, w = scr.getmaxyx()

nfft = w
a = [ 0.0 ] * nfft

def callback(data):
   global a, f
   a = a[1:] + [data[2]]
   f = np.fft.fft(a)

acc = SCL_Reader('gyro', 'sub', [0.0, 0.0, 0.0], callback)

try:
   while 1:  
      sleep(0.05)
      try:
         data = np.real(f)
      except:
         continue
      scr.clear()
      for i in range(len(data) / 2):
         y = int(data[i])
         if y >= h:
             y = h
         for j in range(y):
            scr.addch(h - 1 - j, i * 2, curses.ACS_CKBOARD)
            scr.addch(h - 1 - j, i * 2 + 1, curses.ACS_CKBOARD)
      scr.refresh()
except:
   pass
curses.endwin()
