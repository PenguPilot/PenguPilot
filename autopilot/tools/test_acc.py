
from math import sqrt
from sys import stdin

for line in stdin.readlines():
   x, y, z = map(float, line.split(' '))
   print sqrt(x * x + y * y + z * z)

