
class ControlMode(object):

   pass


class GyroMode(ControlMode):
   
   def __init__(self, name, pitch_roll_p, yaw_p):
      self.name = name
      self.pitch_roll_p = pitch_roll_p
      self.yaw_p = yaw_p

   def __repr__(self):
      return 'GyroMode(%s %f %f)' % (self.name, self.pitch_roll_p, self.yaw_p)

