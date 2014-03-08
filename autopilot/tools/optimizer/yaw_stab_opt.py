
from random import choice, uniform
from scl import generate_map
from opcd_interface import OPCD_Interface
from msgpack import loads
import sys


MUTATION_RATE = 0.2
NUM_SAMPLES = 1000

gates = generate_map('optimizer')
opcd = OPCD_Interface(gates['opcd_ctrl'])
debug = gates['blackbox']

from copy import copy
param_names = ['kp', 'ki', 'kii', 'kd']
prefix = 'pilot.controllers.stabilizing.yaw_'
vec = [ opcd.get(prefix + n) for n in param_names]
fit_best = sys.float_info.max
vec_best = vec
while True:
   # apply mutation:
   new = map(lambda x: x * uniform(1.0 - MUTATION_RATE, 1.0 + MUTATION_RATE), vec)
   i = choice(range(4))
   vec[i] = new[i] #map(lambda x: x * uniform(1.0 - MUTATION_RATE, 1.0 + MUTATION_RATE), vec)
   
   # send new gains to opcd:
   for i, n in zip(range(4), param_names):
      opcd.set(prefix + n, vec[i])
   
   # read data from autopilot and compute fitness:
   fit = 0.0
   for _ in range(NUM_SAMPLES):
      array = loads(debug.recv())
      gyro = float(array[3])
      fit += (0.0 - gyro) ** 2
   fit /= NUM_SAMPLES
   print 'computed fitness:', fit
   
   if fit < fit_best:
      # fitness has increased, print and commit to opcd
      print 'new best fitness:', vec, fit
      opcd.persist()
      fit_best = fit
      vec_best = vec
   else:
      # we did not improve;
      # use best vector as search starting point
      vec = copy(vec_best)

