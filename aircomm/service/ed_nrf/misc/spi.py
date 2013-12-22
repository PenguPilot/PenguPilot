import array


class nRF_on_EDACM():
	Device=None
	Debug=False
	
	def __init__(self,tty_dev):
		import serial
		self.Device=serial.Serial(tty_dev)
	
	def msg(self,data):
		if self.Device:
			if self.Debug:
				print "CS=0"
			self.Device.setBreak(False)
			
			a_data=array.array('B')
			a_data.extend(data)
			self.Device.write(a_data.tostring())
			if self.Debug:
				print "TX: "+str(data)
			self.Device.flushInput()
			data=self.Device.read(len(data))
			a_data=list(array.array('B',(data)))
			if self.Debug:
				print "RX: "+ str(a_data)
			
			self.Device.setBreak(True)

			if self.Debug:
				print "CS=1"
				print ""
			
			return a_data
			
	def setCE(self,value):
		if self.Device:
			self.Device.setDTR(value)
			if self.Debug:
				print "CE=%i"%value
			
	
	