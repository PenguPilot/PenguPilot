
from time import time


class ControlSensorBits:

   def __init__(self):
      self.cs_list = ['GYRO_3D', 'ACC_3D', 'MAG_3D', 'PRESSURE_ABS', 'PRESSURE_DIFF', 'GPS',
         'OPTICAL_FLOW', 'COMPUTER_VISION', 'LASER_SCANNER', 'VICON_LEICA', 'ANGLE_RATE_CONTROL',
         'ATTITUDE_CTRL', 'YAW_CTRL', 'ALTITUDE_CTRL', 'XY_CTRL', 'MOTOR_CTRL']

   def bits(self, flag_names):
      bits = 0
      for i in range(0, len(self.cs_list)):
         if self.cs_list[i] in flag_names:
            bits |= 1 << i
      return bits


class MAVLink_Interface:

   def __init__(self, mavio):
      self.mavio = mavio
   
   def _uptime(self):
      uptime_file = open("/proc/uptime")
      uptime_list = uptime_file.read().split()
      uptime_file.close()
      return float(uptime_list[0])

   def _uptime_ms(self):
      return int(self._uptime() * 1000.0)

   def _uptime_us(self):
      return long(self._uptime() * 1000000.0)

   def _time_ms(self):
      return int(time() * 1000.0)

   def _time_us(self):
      return long(time() * 1000000.0)

   def send_local_position(self, x, y, z, vx, vy, vz):
      '''
      filtered local position (e.g. fused GPS and accelerometers).
      Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
      '''
      self.mavio.mav.local_position_ned_send(self._uptime_ms(),
         float(x), # X Position in m
         float(y), # Y Position in m
         float(z), # Z Position in m
         float(vx), # X Speed in m/s
         float(vy), # Y Speed in m/s 
         float(vz)) # Z Speed in m/s

   def send_attitude(self, roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed):
      self.mavio.mav.attitude_send(self._uptime_ms(),
         float(roll), # Roll angle (rad)
         float(pitch), # Pitch angle (rad)
         float(yaw), # Yaw angle (rad)
         float(roll_speed), # Roll angular speed (rad/s)
         float(pitch_speed), # Pitch angular speed (rad/s)
         float(yaw_speed)) # Yaw angular speed (rad/s)

   def send_gps_position(self, fix_type, lon_deg, lat_deg, alt_m,
                         hdop_m = None, vdop_m = None, speed_m_s = None,
                         course_deg = None, sats_visible = None):
      params = [self._time_us(), # Timestamp (microseconds since UNIX epoch or microseconds since system boot)
         fix_type, # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
         long(lat_deg * 1.0e7), # Latitude in 1E7 degrees
         long(lon_deg * 1.0e7), # Longitude in 1E7 degrees
         int(alt_m * 1.0e3) # Altitude in 1E3 meters (millimeters) above MSL
      ]
      if hdop_m:
         params.append(int(hdop_m * 100.0)) # GPS HDOP horizontal dilution of position in cm (m*100).
      else:
         params.append(65535) # If unknown, set to: 65535
      if vdop_m:
         params.append(int(vdop_m * 100.0)) # GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
      else:
         params.append(65535) # If unknown, set to: 65535
      if hdop_m:
         params.append(int(speed_m_s * 100.0)) # GPS ground speed (m/s * 100).
      else:
         params.append(65535) # If unknown, set to: 65535
      if course_deg:
         params.append(int(course_deg * 100.0)) # Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees.
      else:
         params.append(65535) # If unknown, set to: 65535
      if sats_visible:
         params.append(sats_visible) # Number of satellites visible.
      else:
         params.append(255) # If unknown, set to 255
      self.mavio.mav.gps_raw_int_send(*params)

