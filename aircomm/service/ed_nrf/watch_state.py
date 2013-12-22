import sys
import time
from dev.cdc import CDC_nRF


myDev=CDC_nRF(sys.argv[1])
print myDev.setPower(1)

while True:
	print "0x%02x" % myDev.getStatus()
	time.sleep(1)
