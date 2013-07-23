
from random import uniform
from scl import generate_map
from opcd_interface import OPCD_Interface
import sys

MUTATION_RATE = 0.01
NUM_SAMPLES = 10000

gates = generate_map('optimizer')
opcd = OPCD_Interface(gates['opcd_ctrl'])
debug = gates['blackbox']


param_names = ['kp', 'ki', 'kii', 'kd']
prefix = 'pilot.controllers.stabilizing.yaw_'
vec = [ opcd.get(prefix + n) for n in param_names]
fit_best = sys.float_info.max
vec_best = vec
while True:
   # apply slight mutation:
   vec = map(lambda x: x * uniform(1.0 - MUTATION_RATE, 1.0 + MUTATION_RATE), vec)
   # apply new gains to UAV:
   for i, n in zip(range(4), param_names):
      opcd.set(prefix + n, vec[i])
   # read data and compute fitness:
   fit = 0.0
   for _ in range(NUM_SAMPLES):
      print debug.recv().unpack()
      fit += (pitch_sp - pitch_rate) ** 2 + (roll_sp - roll_rate) ** 2
   # check if the fitness has increased:
   if fit < fit_best:
      # yes, print and commit to opcd
      print vec, fit
      opcd.persist()
      fit_best = fit
      vec_best = vec

