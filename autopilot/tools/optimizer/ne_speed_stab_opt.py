
from random import choice, uniform
from scl import generate_map
from opcd_interface import OPCD_Interface
from msgpack import loads
import sys


MUTATION_RATE = 0.1
NUM_SAMPLES = 2000

gates = generate_map('optimizer')
opcd = OPCD_Interface(gates['opcd_ctrl'])
debug = gates['blackbox']

from copy import copy
param_names = ['p', 'i', 'd']
prefix = 'pilot.controllers.ne_speed.'
vec = [ opcd.get(prefix + n) for n in param_names]
fit_best = sys.float_info.max
vec_best = vec
while True:
   # apply mutation:
   new = map(lambda x: x * uniform(1.0 - MUTATION_RATE, 1.0 + MUTATION_RATE), vec)
   i = choice(len(param_names))
   vec[i] = new[i]
   
   # send new gains to opcd:
   for i, n in zip(range(4), param_names):
      opcd.set(prefix + n, vec[i])
   
   # read data from autopilot and compute fitness:
   fit = 0.0
   for _ in range(NUM_SAMPLES):
      array = loads(debug.recv())
      n_spd_err = float(array[28])
      e_spd_err = float(array[29])
      fit += sqrt(n_spd_err ** 2 + e_spd_err ** 2)
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

