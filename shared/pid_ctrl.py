
from geomath import sym_limit
from time import time


class PID_Ctrl:

   def __init__(self, alt_err_func = None):
      self.p = 0.0
      self.i = 0.0
      self.max_sum_err = 0.0
      self.sum_err = 0.0
      self.d = 0.0
      self.prev_err = 0.0
      self.int_en = False
      self.prevtm = time()
      self.err = 0.0
      self.alt_err_func = alt_err_func

   def control(self, pv, sp, sp_deriv = None):
      self.currtm = time()
      dt = self.currtm - self.prevtm
      if self.alt_err_func:
         err = self.alt_err_func(sp, pv)
      else:
         err = sp - pv
      self.err = err
      ctrl = self.p * err
      if self.int_en:
         self.sum_err += err * dt
         if self.max_sum_err:
            self.sum_err = sym_limit(self.sum_err, self.max_sum_err)
      ctrl += self.i * self.sum_err
      if sp_deriv:
         ctrl += self.d * -sp_deriv
      else:
         if dt > 0:
            ctrl += self.d * (self.prev_err - err) / dt
         self.prev_err = err;
      return ctrl;

   def reset(self):
      self.sum_err = 0.0

