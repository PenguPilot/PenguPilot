# -*- coding: utf-8 -*-
"""
	listen via the command interface
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
myDev=CDC_nRF(sys.argv[1])

#myDev.setRX_Address(0,[0xc0,0x1d,0xbe,0xef,0x01])
while True:
	print myDev.receiveMessage()
	time.sleep(1)
