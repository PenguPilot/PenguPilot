import time
from dev.nrfeep import EEP_nRF

import sys

print sys.argv[1]
myEEP=EEP_nRF(sys.argv[1])

myEEP.parse()

myEEP.dumpData()
