import time
from dev.cdc import CDC_nRF


myDev=CDC_nRF('/dev/ttyACM0')

a=0
while True:
	a=(a+1)&0xff
	print a,myDev.Echo(a,range(1,a+1))
	time.sleep(1)
