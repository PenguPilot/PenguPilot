# -*- coding: utf-8 -*-
"""
	enter DFU-bootloader
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-12-19"
__version__	= "0.1.0"

import os
import sys

sys.path=\
	[os.path.dirname(os.path.realpath(__file__))+'/../']+sys.path

from lib.cdc import CDC_nRF


myDev=CDC_nRF('/dev/ttyACM0')


print myDev.enterBootloader()
