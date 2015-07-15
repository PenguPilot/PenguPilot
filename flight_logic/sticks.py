from geomath import deg2rad
from math import sqrt, pow, hypot
from numpy import array
from numpy.linalg import norm


def init(_opcd):
   global opcd
   opcd = _opcd


class Linfunc:

   def __init__(self, x1, y1, x2, y2):
      self.m = (y2 - y1) / (x2 - x1)
      self.n = y1 - self.m * x1

   def calc(self, x):
      return self.m * x + self.n


def pitch_roll_speed_max():
   return deg2rad(opcd['sticks.pitch_roll_speed_max'])


def pitch_roll_angle_max():
   return deg2rad(opcd['sticks.pitch_roll_angle_max'])


def vert_speed_max():
   return opcd['sticks.vert_speed_max']


def vert_deadzone():
   return opcd['sticks.vert_deadzone']


def horiz_speed_max():
   return opcd['sticks.horiz_speed_max']


def horiz_deadzone():
   return opcd['sticks.horiz_deadzone']


def yaw_speed_max():
   return deg2rad(opcd['sticks.yaw_speed_max'])


def yaw_deadzone():
   return opcd['sticks.yaw_deadzone']


def gas_acc_max():
   return opcd['sticks.gas_acc_max']


def rotation():
   return deg2rad(opcd['sticks.rotation'])


def stick_expo(x):
   base = opcd['sticks.expo']
   if base <= 1.0:
      base = 1.0
   scale = 1.0 / (base - 1.0)
   if x >= 0.0:
      return scale * (pow(base, x) - 1.0)
   else:
      return -scale * (pow(base, -x) - 1.0)


def stick_dz(g, d):
   dz_l = -d / 2.0
   dz_r = d / 2.0
   right = Linfunc(0.5, 0.5, dz_r, 0.0)
   left = Linfunc(-0.5, -0.5, dz_l, 0.0)
   if g < dz_l:
      return left.calc(g)
   elif g > dz_r:
      return right.calc(g)
   else:
     return 0.0


def pitch_roll_speed_func(stick):
   return pitch_roll_speed_max() * stick_expo(stick)


def pitch_roll_angle_func(stick):
   return pitch_roll_angle_max() * stick_expo(stick)


def gas_acc_func(stick):
   return gas_acc_max() * stick_expo(stick)


def gas_speed_func(stick):
   return vert_speed_max() * stick_expo(stick)


def pitch_roll_gps_speed_func(sticks):
   expo_sticks = array(map(stick_expo, sticks))
   n = norm(expo_sticks)
   if n > sqrt(2.0):
      expo_sticks *= 1.0 / n
   return expo_sticks * horiz_speed_max()


def pitch_roll_in_deadzone(pitch, roll):
   return hypot(pitch, roll) < horiz_deadzone()


def gas_in_deadzone(gas):
   return fabs(gas) < vert_deadzone()


def gas_speed_deadzone(gas):
   vmax = vert_speed_max()
   return stick_dz(gas, vert_deadzone()) * vmax


def yaw_speed_deadzone(yaw):
   return stick_dz(yaw, yaw_deadzone() * yaw_speed_max())


def channel_to_mode(sw):
   a = 1.0 / 3.0
   b = 2.0 / 3.0
   if sw <= a:
      return 'gyro'   
   elif sw > a and sw < b:
      return 'acc'
   return 'gps'

