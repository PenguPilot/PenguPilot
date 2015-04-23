from scl import scl_get_socket
from msgpack import loads
from cal_math import Calibration
from opcd_interface import OPCD_Interface
from math import isnan

opcd = OPCD_Interface(scl_get_socket('opcd_ctrl', 'req'))
points = []

socket = scl_get_socket('acc_raw', 'sub')
while True:
   try:
      vec = loads(socket.recv())
      points.append(vec)
      print len(points)
   except:
      break

c = Calibration(points)
cal = c.get_cal()
names = ['acc_bias_x', 'acc_bias_y', 'acc_bias_z', 'acc_scale_x', 'acc_scale_y', 'acc_scale_z']
for i in range(len(names)):
   val = float(cal[i])
   if isnan(val):
      print 'bad calibration'
      break
   opcd.set('acc_adc_cal.' + names[i], val)

