import time

import sys
import os

import yaml

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path


from lib.cdc import CDC_nRF
from lib.nrfeep import EEP_nRF

myDev=CDC_nRF(sys.argv[1])

data=yaml.load(sys.stdin.read(1024))

print("== Data from stdin ==")
print(data)
myDev.writeEEPROM(0,data)

print("== Read-Back ==")
print myDev.readEEPROM(0,len(data))
#print myEEP.tobinarray(start=0x00)
