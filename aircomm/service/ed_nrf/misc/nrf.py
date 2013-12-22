class nRF24():
	"""read command to register"""
	CMD_READ_REG				= 0x00
	
	"""write command to register"""
	CMD_WRITE_REG				= 0x20
	
	"""RX payload register address"""
	CMD_RD_RX_PLOAD			= 0x61
	
	"""TX payload register address"""
	CMD_WR_TX_PLOAD			= 0xA0
	
	"""flush TX register command"""
	CMD_FLUSH_TX				= 0xE1
	
	"""flush RX register command"""
	CMD_FLUSH_RX				= 0xE2
	
	"""reuse TX payload register command"""
	CMD_REUSE_TX_PL			= 0xE3
	
	"""No Operation, might be used to read status register"""
	CMD_NOP							= 0xFF


	"""Config"""
	REG_CONFIG					= 0x00
	
	"""Enable Auto Acknowledgment"""
	REG_EN_AA						= 0x01
	
	"""Enabled RX addresses"""
	REG_EN_RXADDR				= 0x02
	
	"""Setup address width"""
	REG_SETUP_AW				= 0x03
	
	"""Setup Auto. Retrans"""
	REG_SETUP_RETR			= 0x04
	
	"""RF channel"""
	REG_RF_CH						= 0x05
	
	"""RF setup"""
	REG_RF_SETUP				= 0x06
	
	"""Status"""
	REG_STATUS					= 0x07
	
	"""Observe TX"""
	REG_OBSERVE_TX			= 0x08
	
	"""Carrier Detect"""
	REG_CD							= 0x09
	
	"""RX address pipe0"""
	REG_RX_ADDR_P0			= 0x0A
	
	"""RX address pipe1"""
	REG_RX_ADDR_P1			= 0x0B
	
	"""RX address pipe2"""
	REG_RX_ADDR_P2			= 0x0C
	
	"""RX address pipe3"""
	REG_RX_ADDR_P3			= 0x0D
	
	"""RX address pipe4"""
	REG_RX_ADDR_P4			= 0x0E
	
	"""RX address pipe5"""
	REG_RX_ADDR_P5			= 0x0F
	
	"""TX address"""
	REG_TX_ADDR					= 0x10
	
	"""RX payload width, pipe0"""
	REG_RX_PW_P0				= 0x11
	
	"""RX payload width, pipe1"""
	REG_RX_PW_P1				= 0x12
	
	"""RX payload width, pipe2"""
	REG_RX_PW_P2				= 0x13
	
	"""RX payload width, pipe3"""
	REG_RX_PW_P3				= 0x14
	
	"""RX payload width, pipe4"""
	REG_RX_PW_P4				= 0x15
	
	"""RX payload width, pipe5"""
	REG_RX_PW_P5				= 0x16
	
	"""FIFO Status Register"""
	REG_FIFO_STATUS			= 0x17


	SPI=None
	
	
	def __init__(self,spi_obj):
		self.SPI=spi_obj

	def readRegister(self,address):
		data=self.SPI.msg([self.CMD_READ_REG|address,0])
		return data[1]

	def writeRegister(self,address,value_s):
		if type(value_s)==list:
			self.SPI.msg([self.CMD_WRITE_REG|address]+value_s)
		elif type(value_s)==int:
			self.SPI.msg([self.CMD_WRITE_REG|address,value_s])
		

	def readStatus(self):
		return self.readRegister(self.REG_STATUS)
		
		
	def configTX(self,address):
		self.SPI.setCE(0)
  	self.writeRegister(self.REG_TX_ADDR, address);

		#RX_Addr0 same as TX_Adr for Auto.Ack
		self.writeRegister(self.REG_RX_ADDR_P0, address)
		
		# Writes data to TX payload
  	self.writeRegister(WR_TX_PLOAD, tx_buf)

  	#Enable Auto.Ack:Pipe0
  	self.writeRegister(self.REG_EN_AA, 0x01)
  	
  	#Enable Pipe0
  	self.writeRegister(self.REG_EN_RXADDR, 0x01)
  	
  	#500us + 86us, 10 retrans...
  	self.writeRegister(self.REG_SETUP_RETR, 0x1a)
  	
  	#Select RF channel 40
  	self.writeRegister(self.REG_RF_CH, 40)
  	
  	#TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  	self.writeRegister(self.REG_RF_SETUP, 0x07)
  	
  	#Set PWR_UP bit, enable CRC(2 bytes) & Prim:TX. MAX_RT & TX_DS enabled..
  	self.writeRegister(self.REG_CONFIG, 0x0e)

		self.SPI.setCE(1)

	def configRX(self,address):
		self.SPI.setCE(0)
		#Use the same address on the RX device as the TX device
		self.writeRegister(self.REG_RX_ADDR_P0, address)

		#Enable Auto.Ack:Pipe0
		self.writeRegister(self.REG_EN_AA, 0x01)
		
		#Enable Pipe0
		self.writeRegister(self.REG_EN_RXADDR, 0x01)
		
		#Select RF channel 40
		self.writeRegister(self.REG_RF_CH, 40)
		
		#Select same RX payload width as TX Payload width
		TX_PLOAD_WIDTH=0
		self.writeRegister(self.REG_RX_PW_P0, TX_PLOAD_WIDTH)
		
		#TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
		self.writeRegister(self.REG_RF_SETUP, 0x07)
	
		#Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..
		self.writeRegister(self.REG_CONFIG, 0x0f)

		#Set CE pin high to enable RX device
		self.SPI.setCE(1)

		# This device is now ready to receive one packet of 16 bytes payload from a TX device sending to address
		# '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
		

	def clearIF(self):
		#clear interrupt flag(TX_DS)
		status=self.readStatus()
		self.writeRegister(REG_STATUS,status)
