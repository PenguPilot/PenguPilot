import time
from dev.cdc import CDC_nRF


myDev=CDC_nRF('/dev/ttyACM0')

t_start=time.time()
n_regs=10
for reg in range(0,n_regs):
	print myDev.readRegister(reg)
t_end=time.time()

print "Reading %i registers took %fs" % (n_regs,t_end-t_start)

	
