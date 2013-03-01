import sys
import time
from dev.nrfeep import EEP_nRF
from dev.cdc import CDC_nRF


myDev=CDC_nRF(sys.argv[1])

data=myDev.readEEPROM()
print data
if data:
	myEEP=EEP_nRF()
	myEEP.parse(data)

	myEEP.dumpData()

#print sys.argv[1]
#myEEP=EEP_nRF(sys.argv[1])

#myEEP.parse()

#myEEP.dumpData()
