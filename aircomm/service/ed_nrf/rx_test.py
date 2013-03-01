import time
from dev.cdc import CDC_nRF


myDev=CDC_nRF('/dev/ttyACM3')

while True:
	print myDev._rx(10)
	time.sleep(1)
	
