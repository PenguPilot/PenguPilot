from dev.cdc import CDC_nRF
import sys
import time

myDev=CDC_nRF(sys.argv[1])
myDev.setPower(1)

#myDev.setRX_Address(0,[0xc0,0x1d,0xbe,0xef,0x01])
while True:
	print myDev.receiveMessage()
	time.sleep(1)
