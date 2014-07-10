
from bridge import Bridge
from threading import Thread, Timer
from core_pb2 import MonData
from math import sqrt
from mavlinkv10 import *
from psutil import cpu_percent
from util.mavlink_util import ControlSensorBits
import time
from util.load import LoadReader
from util.power import PowerReader


class CoreBridge(Bridge):
 
   def __init__(self, socket_map, mav_iface, send_interval, dispatcher):
      Bridge.__init__(self, socket_map, mav_iface, send_interval)
      self.dead = False
      recv_thread = Thread(target = self._receive)
      recv_thread.daemon = True
      send_thread = Thread(target = self._send)
      send_thread.daemon = True
      self.dispatcher = dispatcher
      self.auto_mode_flags = MAV_MODE_FLAG_SAFETY_ARMED \
         | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED \
         | MAV_MODE_FLAG_STABILIZE_ENABLED \
         | MAV_MODE_FLAG_GUIDED_ENABLED \
         | MAV_MODE_FLAG_AUTO_ENABLED

      Bridge.__init__(self, socket_map, mav_iface, send_interval)
      self.csb = ControlSensorBits()
      self.sensors_present = self.csb.bits(['GYRO_3D', 'ACC_3D', 'MAG_3D',
         'PRESSURE_ABS', 'GPS', 'ANGLE_RATE_CONTROL', 'ATTITUDE_CTRL',
         'YAW_CTRL', 'ALTITUDE_CTRL', 'XY_CTRL', 'MOTOR_CTRL'])
      self.sensors_enabled = self.sensors_present
      self.sensors_health = self.sensors_present
      self._load_reader = LoadReader()
      self._power_reader = PowerReader(socket_map['power_mon'])
      recv_thread.start()
      send_thread.start()
      self._load_reader.start()
      self._power_reader.start()
      self._power_reader.wait_data()

   def _dead(self):
      self.dead = True

   def _receive(self):
      mon = MonData()
      socket = self.socket_map['core_mon']
      last_read = time.time()
      while True:
         if not self.dead:
            timer = Timer(1.0, self._dead)
            timer.start()
         str = socket.recv()
         mon.ParseFromString(str)
         self.mon = mon
         if not self.dead:
            timer.cancel()
         self.dead = False

   def _send(self):
      while True:
         time.sleep(self.send_interval)
         try:
            mon = self.mon
            self.mav_iface.send_local_position(mon.x, mon.y, -mon.z,
               mon.x_speed, mon.y_speed, -mon.z_speed)
            self.mav_iface.send_attitude(mon.roll, mon.pitch, mon.yaw,
               mon.roll_speed, mon.pitch_speed, mon.yaw_speed)
            ground_speed = sqrt(mon.x_speed ** 2 + mon.y_speed ** 2)
            airspeed = 0.0 # TODO: fix me
            throttle = 0.5 # todo: fix me
            self.mav_iface.mavio.mav.vfr_hud_send(airspeed, ground_speed,
               mon.yaw, throttle, -mon.z, -mon.z_speed)
            voltage = self._power_reader.power.voltage
            current_battery = 0
            battery_remaining = -1
            drop_rate_comm = int(10000.0 * self.dispatcher.loss_rate)
            errors_comm = 0
            errors_count1 = 0
            errors_count2 = 0
            errors_count3 = 0
            errors_count4 = 0
         
            if self.dead:
               sensors_enabled = 0
               self.auto_mode_flags &= ~(MAV_MODE_FLAG_STABILIZE_ENABLED \
                  | MAV_MODE_FLAG_GUIDED_ENABLED \
                  | MAV_MODE_FLAG_AUTO_ENABLED)
            else:
               sensors_enabled = self.sensors_enabled
               self.auto_mode_flags |= (MAV_MODE_FLAG_STABILIZE_ENABLED \
                  | MAV_MODE_FLAG_GUIDED_ENABLED \
                  | MAV_MODE_FLAG_AUTO_ENABLED)
        
            self.mav_iface.mavio.mav.heartbeat_send(MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, self.auto_mode_flags, 0, MAV_STATE_ACTIVE)
            self.mav_iface.mavio.mav.sys_status_send(self.sensors_present, sensors_enabled, 0,
               self._load_reader.load * 10, voltage * 1000, current_battery, battery_remaining, drop_rate_comm, 
               errors_comm, errors_count1, errors_count2, errors_count3, errors_count4)
         except Exception, e:
            print str(e)

