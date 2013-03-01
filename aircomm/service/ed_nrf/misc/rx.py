import nrf
import spi
import time

nRF=nrf.nRF24(spi.nRF_on_EDACM('/dev/ttyACM0'))
#nRF.SPI.Debug=True

nRF.configRX(address=[0x34,0x43,0x10,0x10,0x01])
  
while True:
	state=nRF.readStatus()
	print "State:", state
	time.sleep(0.1)
