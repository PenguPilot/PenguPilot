import time

import sys

from dev.cdc import CDC_nRF
from dev.nrfeep import EEP_nRF

myDev=CDC_nRF(sys.argv[1])
myEEP=EEP_nRF(sys.argv[2])

data=list(myEEP.tobinarray(start=0x00))
print data
myDev.writeEEPROM(0,data)


print myDev.readEEPROM(0,len(data))
#print myEEP.tobinarray(start=0x00)