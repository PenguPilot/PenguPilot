
from math import pi

def limit(n, minn, maxn):
   return min(max(n, minn), maxn)

def sym_limit(n, m):
   return clamp(n, -m, m)

def angles_diff(a, b):
  d = b - a
  if d <= -pi:
     d += 2.0 * pi
  if d >= pi:
     d -= 2.0 * pi
  return d


