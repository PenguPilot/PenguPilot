import time

import serial

bus=serial.Serial('/dev/ttyACM3',timeout=1)
bus.setDTR(False)

while True:
	print '==rx=='
	print bus.read(1024)
	bus.write("Test\n\r")
	time.sleep(1)
	
