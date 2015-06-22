
from opcd_interface import *
from time import sleep

r = OPCD_Subscriber()
while True:
   sleep(5)
   print r['icarus.takeoff.standard_rel_z']
