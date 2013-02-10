

def walk_rec(o, lvl):
   try:
      for i in vars(o).items():
         print ' ' * 3 * lvl + str(i[0]) + ':'
         if i[1].__class__.__name__ in ['float64', 'str', 'int', 'ndarray', 'float']:
            print ' ' * 3 * (lvl + 1) + str(i[1])
         walk_rec(i[1], lvl + 1)
   except:
      pass


def walk(o):
   walk_rec(o, 0)

