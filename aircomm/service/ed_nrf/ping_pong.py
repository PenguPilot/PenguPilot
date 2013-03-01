from dev.cdc import CDC_nRF
import sys
import time

myDev=CDC_nRF(sys.argv[2])
myDev.setPower(1)

myDev._bus.setDTR(True)

if sys.argv[1]=='--server':
	print "Starting as server..."
	myDev._bus.timeout=None;
	while True:
		myDev._bus.read()
		myDev._bus.flushInput()
		#time.sleep(0.1)
		myDev._bus.write('Pong!\n\r')
		print "Pong!"
else:
	print "Starting as client..."
	sys.stdout.write("  Max |   Min |   AVG |  last | \n")
	t_ms_vals=[]
	myDev._bus.timeout=1;
	while True:
		t_start=time.time()
		myDev._bus.write('Ping?\n\r')
		myDev._bus.read()
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
