
from ed_nrf.dev.cdc import CDC_nRF
from time import sleep


class Interface:

   def __init__(self, dev_path):
      self.nrf = CDC_nRF(dev_path)
      self.nrf.setPower(True) # enable power
      self.nrf._bus.setDTR(True) # enable transparent mode
      self.nrf._bus.timeout = None # disable read/write timeouts

   def send(self, data):
      sleep(0.01)
      self.nrf._bus.write(data)

   def receive(self):
      return self.nrf._bus.read()

