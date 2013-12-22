import nrf
import spi
import time

nRF=nrf.nRF24(spi.nRF_on_EDACM('/dev/ttyACM0'))
#nRF.SPI.Debug=True

nRF.configTX(address=[0x34,0x43,0x10,0x10,0x01])
  
while True:
	nRF.write([1,2,3,4,5,6,7])
	nRF.clearIF()
	time.wait(0.5);

	state=nRF.readStatus()
	print "State:", state
	time.sleep(0.1)
