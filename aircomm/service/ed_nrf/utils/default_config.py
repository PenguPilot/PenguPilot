# -*- coding: utf-8 -*-
"""
	default ee-prom setup
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-11-30"
__version__	= "0.1.0"

import os
import sys
import time

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path

from lib.cdc import CDC_nRF

myDev=CDC_nRF(sys.argv[1])
print myDev.setDefaultPower(1)
print myDev.setDefaultTXMode(0)
myDev._bus.setDTR(False)
