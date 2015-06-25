

from time import time


class PID:
    
   def __init__(self):
      self.Kp = 0.0
      self.Kd = 0.0
      self.Ki = 0.0
      self.prev_err = 0.0
      self.Ci = 0.0
      self.prevtm = time()

   def reset():
      self.Ci = 0.0

   def run(self, error):
      self.currtm = time()               # get t
      dt = self.currtm - self.prevtm          # get delta t
      de = error - self.prev_err              # get delta error

      Cp = self.Kp * error               # proportional term
      self.Ci += error * dt              # integral term

      Cd = 0
      if dt > 0:                          # no div by zero
         Cd = de / dt                     # derivative term

      self.prevtm = self.currtm               # save t for next pass
      self.prev_err = error                   # save t-1 error

      return Cp + (self.Ki * self.Ci) + (self.Kd * Cd)
 
