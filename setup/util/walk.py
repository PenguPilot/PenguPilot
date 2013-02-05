

def walk_rec(o, lvl):
   try:
      for i in vars(o).items():
         print ' ' * 3 * lvl + str(i[0]),
         if i[1].__class__.__name__ not in ['str', 'int', 'list', 'ndarray', 'float']:
            print i[1].__class__.__name__
         else:
            print i[1]
         walk_rec(i[1], lvl + 1)
   except:
      pass


def walk(o):
   print o.__class__.__name__
   walk_rec(o, 1)

