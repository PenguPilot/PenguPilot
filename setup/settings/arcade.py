

from math import pi
from tools.geometry import *
from hardware import *


class ARCADE_Common(object):
   
   '''common ARCADE set-up, mostly hardware platform related'''

   def __init__(self, cap):
      # always required:
      self.i2c_bus = I2C_Bus("/dev/i2c-3")
      self.marg = DrotekMARG(self.i2c_bus)
      self.rc = ACT_DSL('/dev/ttyO0')
      # optional devices:
      if 'baro' in cap:
         self.baro = MS5611(self.i2c_bus)
      if 'gps' in cap:
         self.gps = SerialGPS("/dev/ttyO2")
      if 'ultra' in cap:
         self.ultra = I2CXL(self.i2c_bus)
      if 'voltage' in cap:
         self.voltage = MADC(7, 'lambda x: (x - 56.0) / 134.0')
      if 'current' in cap:
         self.current = MADC(2, 'lambda x: (2.5 - x / 1024.0) / 0.028')


class ARCADE_Quad(ARCADE_Common):

   def __init__(self, motor_type, cap):
      ARCADE_Common.__init__(self, cap)

      # stabilizing/rate controller and feed forward:
      self.jxx_jyy = 0.0097
      self.jzz = 0.45 * 1.273177e-002
      self.att_kd = 0.02352953418
      self.att_kp = 0.32876523028
      self.att_ki = 2.20026754887
      self.att_kii = 4.37086296837
      self.yaw_kp = 0.108
      self.yaw_kd = 0.00648
      self.yaw_ki = 0.45
      self.filt_fg = 10.0
      self.filt_d = 0.95
      self.tmc = 0.06
      self.rpm_min = 1340.99
      self.rpm_max = 7107.6
      self.thrust_max = 10.0 # N, per actuator
      self.weight = 1.0 # Kg

      # inverse coupling:
      matrix = inv_coupling_matrix_4(0.2025, 1.5866e-007, 4.0174e-009)
      self.inv_coupling = inv_coupling_matrix_to_config(matrix)
      self.process_noise = 1.0e-6

      # set up motors:
      if motor_type == 'holger':
         motor_addrs = [0x29, 0x2a, 0x2b, 0x2c]
         self.motors = [HolgerMotor(self.i2c_bus, addr) for addr in motor_addrs]
      elif motor_type == 'pwm_esc':
         pass
      else:
         raise ValueError('unknown motor type')

