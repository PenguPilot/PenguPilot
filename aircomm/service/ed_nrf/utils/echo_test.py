import os
import sys
import time
sys.path=\
        [os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path



from lib.cdc import CDC_nRF


myDev=CDC_nRF('/dev/ttyACM0')

a=0
while True:
	a=(a+1)&0xff
	print a,myDev.Echo(a,range(1,a+1))
	time.sleep(1)
