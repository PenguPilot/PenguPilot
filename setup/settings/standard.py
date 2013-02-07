

class Highlevel_Control(object):

   '''hopefully, these are independent of the lower control system's parameters'''

   def __init__(self):
      # attitude position controller:
      self.att_p = 3.0
      self.att_i = 0.0005
      self.att_i_max = 1.0
      self.att_d = 0.1
      self.att_pitch_bias = 0.0
      self.att_roll_bias = 0.0
      
      # yaw position controller:
      self.yaw_pos_p = 0.24
      self.yaw_pos_d = 0.1
      self.yaw_pos_lim = 0.2

      # GPS speed controller:
      self.xy_speed_p = 0.4

      # altitude position controller:
      self.alt_p = 0.1
      self.alt_i =  0.0005
      self.alt_i_max = 1.0
      self.alt_d = 0.1

      # navigation position controller:
      self.xy_pos_v_min = 0.5
      self.xy_pos_v_std = 2.0
      self.xy_pos_v_max = 3.5
      self.xy_pos_sqrt_shift = 3.0
      self.xy_pos_sqrt_scale = 0.25
      self.xy_pos_square_shift = 0.5
      self.xy_pos_i = 0.015
      self.xy_pos_i_max = 5.0
      self.xy_pos_ortho_p = 0.0


class Calibration(object):
   
   def __init__(self):
      from tools.geometry import vector_to_config
      self.acc_biases = vector_to_config([ 0.0 ] * 3)
      self.mag_biases = vector_to_config([ 0.0 ] * 3)
      self.acc_scales = vector_to_config([ 1.0 ] * 3)
      self.mag_scales = vector_to_config([ 1.0 ] * 3)



class Behavior(object):

   def __init__(self):
      self.takeoff_z = 3.0
      self.landing_speed = 0.5
      self.landing_z_motors_off = 0.4


class Safety(object):

   def __init__(self):
      self.gps_min_takeoff_sats = 4


class StandardSettings(object):

   def __init__(self):
      self.hl_control = Highlevel_Control()
      self.calibration = Calibration()
      self.behavior = Behavior()
      self.safety = Safety()



class Settings(StandardSettings):

   def __init__(self):
      StandardSettings.__init__(self)
      
      # select and configure platform:
      cap = ['ultra', 'gps', 'baro', 'voltage', 'current']
      self.platform = ARCADE_Quad('holger', cap)
      self.battery = Battery(4, 6.6, 3.3)

