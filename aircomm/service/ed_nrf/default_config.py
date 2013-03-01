import sys
import time
from dev.cdc import CDC_nRF


myDev=CDC_nRF(sys.argv[1])
print myDev.setDefaultPower(1)
myDev._bus.setDTR(False)
