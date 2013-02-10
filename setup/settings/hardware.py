

class Battery(object):

   def __init__(self, cells, capacity, undervoltage):
      self.cells = cells
      self.capacity = capacity
      self.undervoltage = 3.3


class SerialGPS(object):

   def __init__(self, path, protocol = 'NMEA'):
      self.protocol = protocol # might add more here
      self.path = path
      self.noise = 4.0e-3


class ACT_DSL(object):

   def __init__(self, path):
      self.path = path


class MADC(object):

   def __init__(self, channel, func):
      self.path = '/sys/class/hwmon/hwmon0/device/in%d_input' % channel
      self.func = func


class MS5611(object):

   def __init__(self, bus):
      self.bus = bus
      self.noise = 1.0e-2


class I2CXL(object):

   def __init__(self, bus):
      self.bus = bus
      self.range_min = 0.2
      self.range_max = 7.0
      self.noise = 1.0e-2


class MARG(object):

   def __init__(self, orientation = None):
      if orientation:
         self.orientation = orientation


class DrotekMARG(MARG):

   def __init__(self, bus):
      MARG.__init__(self)
      self.bus = bus


class HolgerMotor(object):

   sp_off = 0
   sp_min = 20
   sp_max = 255

   def __init__(self, bus, addr):
      self.addr = addr
      self.bus = bus


class I2C_Bus(object):
   
   def __init__(self, id, path):
      self.id = id
      self.path = path


