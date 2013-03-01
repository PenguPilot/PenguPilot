from dev.cdc import CDC_nRF
import sys
import time

myDev=CDC_nRF(sys.argv[1])
#myDev.setPower(1)

#myDev.setRX_Address(0,[0xc0,0x1d,0xbe,0xef,0x00])
myDev._bus.setDTR(True)
myDev._bus.timeout=None;
while True:
	sys.stdout.write(myDev._bus.read())
	sys.stdout.flush()
	#print myDev._rx(1024)
