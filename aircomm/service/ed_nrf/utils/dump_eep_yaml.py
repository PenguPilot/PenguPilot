import sys
import time
import yaml
import os

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path


from lib.nrfeep import EEP_nRF
from lib.cdc import CDC_nRF


myDev=CDC_nRF(sys.argv[1])

data=myDev.readEEPROM()
print(yaml.safe_dump(data))


