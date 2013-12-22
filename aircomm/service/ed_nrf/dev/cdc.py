# -*- coding: utf-8 -*-
"""
	class to access an nRF via CDC-nRF interface
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2012-04-30"
__version__	= "0.1.1"

import serial
import array
import time

class CDC_nRF:
	
	NRF_FNC_TX_MESSAGE						=	 0
	NRF_FNC_TX_MESSAGE_NOACK			=	16
	NRF_FNC_RX_MESSAGE						=	32
	NRF_FNC_NRF_POWER							=	 1
	NRF_FNC_NRF_DEFAULT_POWER			=	201
	NRF_FNC_GET_NRF_STATUS				=	 2
	NRF_FNC_NRF_RECONFIGURE				=	10
	NRF_FNC_NRF_WRITE_REGISTER		=	11
	NRF_FNC_NRF_READ_REGISTER			=	12
	
	NRF_FNC_NRF_WRITE_TX_ADDRESS	=	20
	NRF_FNC_NRF_READ_TX_ADDRESS		=	21
	
	NRF_FNC_NRF_WRITE_RX_ADDRESS	=	30
	NRF_FNC_NRF_READ_RX_ADDRESS		=	31

	NRF_FNC_WRITE_EEPROM					= 40
	NRF_FNC_READ_EEPROM						= 41
	
	NRF_FNC_ECHO									= 100
	
	DEV_FNC_RESET									=	254
	DEV_FNC_ENTER_BOOTLOADER			= 255

	NRF_RESPONSE_ACK							= 0x80
	NRF_RESPONSE_NACK							= 0x81

	_bus=None
	
	debug=False
	
	def __init__(self,port):
		self._bus=serial.Serial(port,timeout=0.01)
		#self._bus.setDTR(False)
		self._bus.setDTR(False)
		time.sleep(0.2)
		self._bus.flush()
		
	def _rx(self,length=32):
		str_data=self._bus.read(length)
		#self._bus.readinto(data)
		#data=list(array.array('B',(data)))
		data=map(ord,str_data)
		if self.debug:
			print 'rx',data
		return data
		
	def _tx(self,data):
		self._bus.flush()
		if self.debug:
			print 'tx',data
		self._bus.write(serial.to_bytes(data))
	
	def _tx_frame(self,message,rx_length=32):
		self._tx(message)
		return self._rx(rx_length)
	
	def _rx_frame(self,rx_length):
		#TODO: implement
		data=self._rx(rx_length)
		return data
	
	#-----------------------------
	def sendMessage(self,message,ack=True):
		if ack:
			#print message
			ret=self._tx_frame([self.NRF_FNC_TX_MESSAGE]+message,1)
			return True if (len(ret) and (ret[0]==0x80)) else False
		else:
			ret=self._tx_frame([self.NRF_FNC_TX_MESSAGE_NOACK]+message,1)
			return True if (len(ret) and (ret[0]==0x80)) else False
			
	def receiveMessage(self):
		ret=self._tx_frame([self.NRF_FNC_RX_MESSAGE],32)
		if len(ret) == 0:
			return False
		if ret[0]==0x80:
			return ret[2:] #self._rx()
		return False

	def setPower(self,power):
		ret=self._tx_frame([self.NRF_FNC_NRF_POWER,power],2)
		return True if ret[0]==0x80 else False

	def setDefaultPower(self,power):
		ret=self._tx_frame([self.NRF_FNC_NRF_DEFAULT_POWER,power],2)
		return True if ret[0]==0x80 else False

	def getStatus(self):
		ret=self._tx_frame([self.NRF_FNC_GET_NRF_STATUS],2)
		if len(ret)!=2:
			return False
		return ret[1] if (ret[0]==0x80) else False

	def Reconfigure(self):
		ret=self._tx_frame([self.NRF_FNC_NRF_RECONFIGURE]+message,2)
		return True if ret==self.FRAME_ACK else False

	def writeRegister(self,register,value):
		ret=self._tx_frame([self.NRF_FNC_NRF_WRITE_REGISTER,register,value],2)
		return True if ret==self.FRAME_ACK else False

	def Echo(self,length,values):
		ret=self._tx_frame([self.NRF_FNC_ECHO,length]+values,1+len(values))
		return ret[1:] if (ret[0]==0x80) else False

	def readRegister(self,register):
		ret=self._tx_frame([self.NRF_FNC_NRF_READ_REGISTER,register],2)
		#if len(ret)!=3:
		#	return False
		return ret[1] if (ret[0]==0x80) else False

	def readEEPROM(self,addr_start=0,length=64):
		ret=self._tx_frame([self.NRF_FNC_READ_EEPROM,addr_start,length],1+length)
		#if len(ret)!=3:
		#	return False
		return ret[1:] if (ret[0]==0x80) else False
		
	def writeEEPROM(self,addr_start,values):
		ret=[]
		#write 4 bytes at max
		for w in range(0,len(values),4):
			val_block=values[w:w+4]
			#time.sleep(0.1)
			response=self._tx_frame([self.NRF_FNC_WRITE_EEPROM,addr_start+w,len(val_block)]+val_block,1)
			#print val_block
			if (response[0]!=0x80):
				return False
			else:
				ret=ret+[val_block]
		return ret

	def setTX_Address(self,address):
		ret=self._tx_frame([self.NRF_FNC_NRF_WRITE_TX_ADDRESS,0]+address,2)
		return True if ret[0]==0x80 else False

	def setRX_Address(self,fifo_id,address):
		ret=self._tx_frame([self.NRF_FNC_NRF_WRITE_RX_ADDRESS,fifo_id]+address,2)
		return True if ret[0]==0x80 else False

	def getTX_Address(self):
		ret=self._tx_frame([self.NRF_FNC_NRF_READ_TX_ADDRESS,0],7)
		return ret[1:] if ret[0]==0x80 else False

	def getRX_Address(self,fifo_id):
		ret=self._tx_frame([self.NRF_FNC_NRF_READ_RX_ADDRESS,fifo_id],7)
		return ret[1:] if ret[0]==0x80 else False

	def Reset(self):
		ret=self._tx_frame([self.DEV_FNC_RESET],0)
		return None

	def enterBootloader(self):
		ret=self._tx_frame([self.DEV_FNC_ENTER_BOOTLOADER],0)
		return None

	def getAddressLength(self):
		#TODO
		pass

	
	
