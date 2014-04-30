#!/usr/bin/env python
"""
  ___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 Power Management Service
 - monitors system power
 - estimates battery state of charge (SOC)
 - predicts remaining battery lifetime
 - manages main power and lights

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from time import time, sleep
from threading import Thread, Timer
from signal import pause
from smbus import SMBus
from os import system, sep

from misc import *
from power_pb2 import *
from msgpack import dumps
from scl import generate_map
from opcd_interface import OPCD_Interface
from misc import daemonize, Hysteresis, user_data_dir
from hardware import *


class PowerMan:

   def __init__(self, name):
      map = generate_map(name)
      self.ctrl_socket = map['ctrl']
      self.monitor_socket = map['mon']
      self.opcd = OPCD_Interface(map['opcd_ctrl'])
      #bus = SMBus(self.opcd.get('gpio_i2c_bus'))
      #self.gpio_mosfet = GPIO_Bank(bus, self.opcd.get('gpio_i2c_address'))
      #self.power_pin = self.opcd.get('gpio_power_pin')
      self.cells = self.opcd.get('powerman.battery_cells')
      self.low_cell_voltage_idle = self.opcd.get('powerman.battery_low_cell_voltage_idle')
      self.low_cell_voltage_load = self.opcd.get('powerman.battery_low_cell_voltage_load')
      self.battery_current_treshold = self.opcd.get('powerman.battery_current_treshold')
      self.capacity = self.opcd.get('powerman.battery_capacity')
      self.low_battery_voltage_idle = self.cells * self.low_cell_voltage_idle
      self.low_battery_voltage_load = self.cells * self.low_cell_voltage_load
      self.critical = False
      #self.gpio_mosfet.write()
      self.warning_started = False

      # start threads:
      self.standing = True
      self.adc_thread = start_daemon_thread(self.adc_reader)
      self.request_thread = start_daemon_thread(self.request_handler)


   def battery_warning(self):
      # do something in order to indicate a low battery:
      msg = 'CRITICAL WARNING: SYSTEM BATTERY VOLTAGE IS LOW; IMMEDIATE SHUTDOWN REQUIRED OR SYSTEM WILL BE DAMAGED'
      system('echo "%s" | wall' % msg)
      beeper_enabled = self.opcd.get('powerman.beeper_enabled')
      while True:
         if beeper_enabled:
            self.gpio_mosfet.set_gpio(5, False)
            sleep(0.1)
            self.gpio_mosfet.set_gpio(5, True)
            sleep(0.1)
         else:
            sleep(1.0)


   def adc_reader(self):
      platform = self.opcd.get('platform')
      plat_prefix = 'powerman.' + platform
      adc_type = self.opcd.get(plat_prefix + '.adc_type')
      adcs = {'twl4030_madc': TWL4030_MADC, 'ads1x15': ADS1x15_ADC}
      adc_prefix = 'powerman.' + adc_type
      voltage_adc = adcs[adc_type](self.opcd.get(adc_prefix + '.voltage_channel'))
      current_adc = adcs[adc_type](self.opcd.get(adc_prefix + '.current_channel'))
      voltage_lambda = eval(self.opcd.get(adc_prefix + '.adc_to_voltage'))
      current_lambda = eval(self.opcd.get(adc_prefix + '.adc_to_current'))
      vmin = 13.2
      vmax = 16.4
      self.voltage = voltage_lambda(voltage_adc.read())  
      batt = min(1.0, max(0.0, (self.voltage - vmin) / (vmax - vmin)))
      self.consumed = (1.0 - batt) * self.capacity
      hysteresis = Hysteresis(self.opcd.get('powerman.low_voltage_hysteresis'))
      time_prev = time()
      while True:
         sleep(0.2)
         try:
            self.voltage = voltage_lambda(voltage_adc.read())  
            self.current = current_lambda(current_adc.read())
            self.consumed += self.current / (3600 * (time() - time_prev))
            time_prev = time()
            if self.voltage < self.low_battery_voltage_idle and self.current < self.battery_current_treshold or self.voltage < self.low_battery_voltage_load and self.current >= self.battery_current_treshold:
               self.critical = hysteresis.set()
            else:
               hysteresis.reset()
            if self.critical:
               if not self.warning_started:
                  self.warning_started = True
                  start_daemon_thread(self.battery_warning)
            voltage = self.voltage
            current = self.current
            capacity = self.capacity
            consumed = self.consumed
            remaining = self.capacity - self.consumed
            if remaining < 0:
               remaining = 0
            critical = self.critical
            state = [self.voltage,  # 0 [V]
                     self.current,  # 1 [A]
                     remaining,     # 2 [Ah]
                     self.critical] # 3 [Bool]
            print state
         except Exception, e:
            continue
         self.monitor_socket.send(dumps(state))


   def power_off(self):
      self.gpio_mosfet.set_gpio(self.power_pin, False)


   def request_handler(self):
      timeout = self.opcd.get('powerman.power_save_timeout')
      req = PowerReq()
      rep = PowerRep()
      timer = None
      while True:
         rep.status = OK
         try:
            req_data = self.ctrl_socket.recv()
         except:
            sleep(1)
            continue
         try:
            req.ParseFromString(req_data)
         except:
            rep.status = E_SYNTAX
         else:
            try:
               timer.cancel()
            except:
               pass
            if req.cmd == STAND_POWER:
               timer = Timer(timeout, self.power_off)
               timer.start()
            else:
               if self.critical:
                  # flying command is not allowed if battery was critical:
                  rep.status = E_POWER
               else:
                  self.gpio_mosfet.set_gpio(self.power_pin, True)
         self.ctrl_socket.send(rep.SerializeToString())



def main(name):
   PowerMan(name)
   await_signal()

daemonize('powerman', main)

