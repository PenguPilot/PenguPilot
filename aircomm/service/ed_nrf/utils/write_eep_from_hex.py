# -*- coding: utf-8 -*-
"""
	write eeprom hex file into module
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-11-22"
__version__	= "0.1.0"

import os
import sys
import time

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path


from lib.cdc import CDC_nRF
from lib.nrfeep import EEP_nRF

myDev=CDC_nRF(sys.argv[1])
myEEP=EEP_nRF(sys.argv[2])

data=list(myEEP.tobinarray(start=0x00))
data=data+[255,255]
print data

myDev.writeEEPROM(0,data)


print myDev.readEEPROM(0,len(data))
#print myEEP.tobinarray(start=0x00)
