# -*- coding: utf-8 -*-
"""
	class to manage nrf-EEProm settings
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-04-30"
__version__	= "0.1.0"

import nrf_const
from intelhex import IntelHex

class EEP_nRF(IntelHex):
	RegisterValues=None
	finalRegisterValues=None
	
	def parse(self,data=None):
		if not data:
			data=self.tobinarray(start=0x00)
		idx=0
		self.RegisterValues=[]
		self.finalRegisterValues={}
		
		while (len(data)>(idx+2)) and (data[idx]<0xff):
			self.RegisterValues.append([data[idx],data[idx+1]])
			self.finalRegisterValues[data[idx]]=data[idx+1]
			idx=idx+2
			
		self.AddressLength=data[idx+1]
		self.TXAddress=data[idx+2:idx+2+self.AddressLength]
			
	def dumpData(self):
		print "== EEProm Registers =="
		for group in self.RegisterValues:
			print "% 20s = %02x" % (
				nrf_const.REGISTER_NAMES[group[0]],
				group[1]
			)
		print "== EEProm Registers, final values =="
		for address in self.finalRegisterValues:
			print "% 20s = %02x" % (
				nrf_const.REGISTER_NAMES[address],
				self.finalRegisterValues[address]
			)
		print "== Misc =="
		print "% 20s = %02x" % ('AddressLength',self.AddressLength)
		addr=''
		for byte in self.TXAddress:
			addr=addr+("%02x:"%byte)
		addr=addr[:-1]
		print "% 20s = %s"%('Address',addr)
