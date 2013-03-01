
from ed_nrf.dev.cdc import CDC_nRF
from packet.dencode import encode, decode, Packet


class Interface:

   def __init__(self, dev_path):
      self.nrf = CDC_nRF(dev_path)
      self.nrf.setPower(True) # enable power
      self.nrf._bus.setDTR(True) # enable transparent mode
      self.nrf._bus.timeout = None # disable read/write timeouts

   def send(self, data):
      self.nrf._bus.write(raw)

   def receive(self):
      return self.nrf._bus.read()

