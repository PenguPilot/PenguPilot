# -*- coding: utf-8 -*-
"""
	reset cdc-nrf
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-12-19"
__version__	= "0.1.0"

import os
import sys

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path

from lib.cdc import CDC_nRF

import time

myDev=CDC_nRF(sys.argv[1])
print myDev.setPower(1)

while True:
	print "0x%02x" % myDev.getStatus()
	time.sleep(1)
