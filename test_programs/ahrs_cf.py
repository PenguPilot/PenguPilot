from scl import scl_get_socket, SCL_Reader
from math import *


def angles_err(a, b):
  d = b - a
  if d <= -pi:
     d += 2.0 * pi
  if d >= pi:
     d -= 2.0 * pi
  return d;

def unroll_yaw(yaw):
   if yaw < 0:
      yaw += 2 * pi
   if yaw > 2 * pi:
      yaw -= 2 * pi
   return yaw

acc = SCL_Reader('acc', 'sub', [0.0, 0.0, -9.81])
mag = SCL_Reader('mag', 'sub', [1.0, 0.0, 0.0])
s = scl_get_socket('gyro', 'sub')
o = scl_get_socket('orientation2', 'pub')
kp = 1.0
roll_est = 0.0
pitch_est = 0.0
yaw_est = 0.0
while True:
   if kp > 0.002:
      kp -= 0.002
   if kp < 0.001:
      kp = 0.002
   gyro = s.recv()
   roll = atan2(-acc.data[1], -acc.data[2]);
   roll_est += gyro[0] * 0.005
   roll_err = roll_est - roll
   roll_est -= kp * roll_err
   
   pitch = atan2(acc.data[0], -acc.data[2]);
   pitch_est += gyro[1] * 0.005
   pitch_err = pitch_est - pitch
   pitch_est -= kp * pitch_err
   
   xmag = mag.data[0]
   ymag = mag.data[1]
   zmag = mag.data[2]
   
   yaw = atan2(xmag * cos(pitch_est) + zmag * sin(pitch_est), 
               xmag * sin(roll_est) * sin(pitch_est) + ymag * cos(roll_est) - zmag * sin(roll_est) * cos(pitch_est))

   yaw = unroll_yaw(yaw + pi)
   
   yaw_est += gyro[2] * 0.005
   yaw_est = unroll_yaw(yaw_est)
   yaw_err = angles_err(yaw_est, yaw)
   yaw_est -= 0.1 * kp * yaw_err

   o.send([yaw_est, pitch_est, roll_est])
