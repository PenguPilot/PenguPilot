# -*- coding: utf-8 -*-
"""
	raw listen, receive bytes in raw mode
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


myDev=CDC_nRF(sys.argv[2])
myDev.setPower(1)
myDev.setDefaultTXMode(0)

myDev._bus.setDTR(True)

if sys.argv[1]=='--server':
	print "Starting as server..."
	myDev._bus.timeout=None;
	while True:
		char=myDev._bus.read()
		if char:
			print('rx',char+myDev._bus.read(myDev._bus.inWaiting()))
			myDev._bus.flushInput()
		time.sleep(0.1)
		print('tx',"Pong!")
		myDev._bus.write('Pong!')
else:
	print "Starting as client..."
	sys.stdout.write("  Max |   Min |   AVG |  last | \n")
	t_ms_vals=[]
	myDev._bus.timeout=1;
	while True:
		t_start=time.time()
		myDev._bus.write('Ping?')
		time.sleep(0.1)
		char=myDev._bus.read()
		if char:
			print(char+myDev._bus.read(myDev._bus.inWaiting()))
			myDev._bus.flushInput()
		t_end=time.time()
		
		t_ms_vals.append((t_end-t_start)*1000)
		if len(t_ms_vals)>100:
			del(t_ms_vals[0])
		sys.stdout.write(
			"\r% 4i | % 4i | % 4i | % 4i |"%
			(max(t_ms_vals),min(t_ms_vals),sum(t_ms_vals)/len(t_ms_vals),t_ms_vals[-1])
		)
		sys.stdout.flush()
