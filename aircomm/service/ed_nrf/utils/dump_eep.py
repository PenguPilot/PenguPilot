# -*- coding: utf-8 -*-
"""
	dump eeprom data
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-11-22"
__version__	= "0.1.0"

import os
import sys
import time

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path

from lib.nrfeep import EEP_nRF
from lib.cdc import CDC_nRF


myDev=CDC_nRF(sys.argv[1])

data=myDev.readEEPROM()
print data
if data:
	myEEP=EEP_nRF()
	myEEP.parse(data)

	myEEP.dumpData()

#print sys.argv[1]
#myEEP=EEP_nRF(sys.argv[1])

#myEEP.parse()

#myEEP.dumpData()
