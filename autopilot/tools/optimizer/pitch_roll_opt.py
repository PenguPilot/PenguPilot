
from random import uniform
from scl import generate_map
from opcd_interface import OPCD_Interface
from msgpack import loads
import sys
from copy import copy
from time import sleep


MUTATION_RATE = 0.1
NUM_SAMPLES = 1000

gates = generate_map('optimizer')
opcd = OPCD_Interface(gates['opcd_ctrl'])
debug = gates['blackbox']


param_names = ['p', 'i', 'd']
prefix = 'autopilot.controllers.attitude.'
vec = [ opcd.get(prefix + n) for n in param_names]
fit_best = sys.float_info.max
vec_best = copy(vec)
while True:
   # apply mutation:
   vec = map(lambda x: x * uniform(1.0 - MUTATION_RATE, 1.0 + MUTATION_RATE), vec)
   
   # send new gains to opcd:
   for i, n in zip(range(4), param_names):
      opcd.set(prefix + n, vec[i])
   
   # read data from autopilot and compute fitness:
   fit = 0.0
   for _ in range(NUM_SAMPLES):
      array = loads(debug.recv())
      fit += array[35] ** 2 + array[36] ** 2
   fit /= NUM_SAMPLES
   
   if fit < fit_best:
      # fitness has increased, print and commit to opcd
      print 'new best fitness:', vec, fit
      opcd.persist()
      fit_best = fit
      vec_best = copy(vec)
   else:
      # we did not improve;
      # use best vector as search starting point
      vec = copy(vec_best)

