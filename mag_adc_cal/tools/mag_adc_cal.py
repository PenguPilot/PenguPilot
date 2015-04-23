from scl import scl_get_socket
from msgpack import loads
from cal_math import Calibration
from opcd_interface import OPCD_Interface
from math import isnan

opcd = OPCD_Interface(scl_get_socket('opcd_ctrl', 'req'))
points = []

socket = scl_get_socket('mag_raw', 'sub')
while True:
   try:
      vec = loads(socket.recv())
      points.append(vec)
      print len(points)
   except:
      break

c = Calibration(points)
cal = c.get_cal()
names = ['mag_bias_x', 'mag_bias_y', 'mag_bias_z', 'mag_scale_x', 'mag_scale_y', 'mag_scale_z']
for i in range(len(names)):
   val = float(cal[i])
   if isnan(val):
      print 'bad calibration'
      break
   opcd.set('mag_adc_cal.' + names[i], val)

