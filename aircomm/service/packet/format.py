
#
# file: format.py
# purpose: packet payload (de)serialization
# author: Tobias Simon, Ilmenau University of Technology
#


from struct import pack, unpack


class _ser:

   '''
   generic serializer class based on 'struct'
   '''

   id = 0

   def __init__(self, fs):
      self.id = _ser.id
      _ser.id += 1
      self.fs = fs

   def pack(self, *args):
      return pack(self.fs, *args)

   def unpack(self, s):
      return unpack(self.fs, s)


_f3d = 'dd' # 3d float vector


# serializer instances:

gpos = _ser(_f3d) # global position estimate: (lat, lon, alt)
lpos = _ser(_f3d) # local position estimate: (x, y, z)
att = _ser(_f3d)  # euler angles: (yaw, pitch, roll)

