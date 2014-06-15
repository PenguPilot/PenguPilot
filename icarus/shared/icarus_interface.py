

# This file is part of ARCADE's ICARUS component.
# ICARUS stands for: (I)ntelligent (C)ommand (A)rbitration and (R)eaction on (U)nforeseen (S)ituations
#
# The purpose of ICARUS is not only the execution of a given command, but to:
# - perform contraint analysis on parameters: checking speed against actuator limits, checking position against battery capacity constraints
# - manage possible auto-landing sites
# - monitor health and react on different low battery states


from icarus_pb2 import *
from threading import Thread, Event
from time import sleep


class ICARUS_MissionFactory:

   '''
   human- and machine-friendly interface to the UAV's ICARUS system
   '''

   def takeoff(self, **kwargs):
      '''
      take-off

      keyword arguments:
         - z: take-off altitude
         - glob: indicates if z should be interpreted global or local
         - speed: take-off speed. Overrides z speed until take-off is complete
      '''
      z = self._get_and_cast(kwargs, 'z', float, None)
      if z == None and 'glob' in kwargs:
         raise AssertionError('glob provided without z')
      glob = self._get_glob(kwargs)
      speed = self._get_speed(kwargs)
      return self._build_takeoff(z, glob, speed)


   def land(self, speed = None):
      '''
      land
      
      arguments:
         - speed: landing speed. Overrides z speed until landing is complete
      '''
      speed = self._cast(speed, float)
      return self._build_land(speed)



   def stop(self):
      '''
      stop the system when moving
      '''
      return self._build_stop()



   def move_x(self, x, **kwargs):
      '''
      move local in x direction

      arguments:
         x value in meters
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, False)
      return self.move((x, None, None), **kwargs)


   def move_y(self, y, **kwargs):
      '''
      move local in y direction

      arguments:
         y value in meters
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, False)
      return self.move((None, y, None), **kwargs)


   def move_z(self, z, **kwargs):
      '''
      move local in z direction

      arguments:
         z value in meters
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, False)
      return self.move((None, None, z), **kwargs)


   def move_xy(self, x, y, **kwargs):
      '''
      move local in x and y direction

      arguments:
         x value in meters
         y value in meters
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, False)
      return self.move((x, y, None), **kwargs)


   def move_xyz(self, x, y, z, **kwargs):
      '''
      move local in x, y and z direction

      arguments:
         x value in meters
         y value in meters
         z value in meters
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, False)
      return self.move((x, y, z), **kwargs)


   def move_lat(self, lat, **kwargs):
      '''
      move to GPS latitude

      arguments:
         lat: latitude in radians
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, True)
      return self.move((lat, None, None), **kwargs)


   def move_lon(self, lon, **kwargs):
      '''
      move to GPS longitude

      arguments:
         lon: longitude in radians
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, True)
      return self.move((None, lon, None), **kwargs)


   def move_alt(self, alt, **kwargs):
      '''
      move to GPS altitude (altitude above mean sea level (MSL))

      arguments:
         alt: altitude in meters above MSL
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, True)
      return self.move((None, None, alt), **kwargs)


   def move_lon_lat(self, lat, lon, **kwargs):
      '''
      move to GPS longitude, latitude

      arguments:
         lon: longitude in radians
         lat: latitude in radians
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, True)
      return self.move((lon, lat, None), **kwargs)


   def move_gps(self, lat, lon, alt, **kwargs):
      '''
      move to GPS position longitude, latitude, altitude

      arguments:
         lon: longitude in radians
         lat: latitude in radians
         alt: altitude in meters above MSL
      keyword arguments:
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed (ground speed)
         block: indicates if the interface should wait for completion (default: True)
      '''
      self._set_glob(kwargs, True)
      return self.move((lat, lon, alt), **kwargs)


   def move(self, pos, **kwargs):
      '''
      move generic

      arguments:
         pos: tuple containing 3d coordinate (absolute / relative, global / local)
      keyword arguments:
         glob: indicates if pos should be interpreted global (True) or local (default: True)
         rel: indicates, if the movement is incremental (default: False)
         speed: movement speed (ground speed)
         block: indicates if the interface should wait for completion (default: True)
      '''
      pos = tuple(map(lambda x: self._cast(x, float), pos))
      glob = self._get_glob(kwargs)
      rel = self._get_rel(kwargs)
      speed = self._get_speed(kwargs)
      block = self._get_block(kwargs)
      return self._build_move(pos, glob, rel, speed, block)


   # private utility methods:


   def _cast(self, val, cast):
      if val is not None:
         try:
            return cast(val)
         except:
            raise TypeError('cannot cast %s to type: %s' % (str(val), str(cast)))


   def _get_and_cast(self, kwargs, name, cast, default):
      if name in kwargs:
         return self._cast(kwargs[name], cast)
      return default


   def _get_glob(self, kwargs):
      return self._get_and_cast(kwargs, 'glob', bool, False)


   def _get_rel(self, kwargs):
      return self._get_and_cast(kwargs, 'rel', bool, False)


   def _get_speed(self, kwargs):
      return self._get_and_cast(kwargs, 'speed', float, None)


   def _get_block(self, kwargs):
      return self._get_and_cast(kwargs, 'block', bool, True)


   def _set_glob(self, kwargs, val):
      assert 'glob' not in kwargs
      kwargs['glob'] = val

   
   def _set_rel(self, kwargs, val):
      assert 'rel' not in kwargs
      kwargs['rel'] = val


   def _build_takeoff(self, z, glob, speed):
      req = IcarusReq()
      req.type = TAKEOFF
      if not z is None:
         req.takeoff_data.z = z
      if not glob is None:
         req.glob = glob
      if not speed is None:
         req.speed = speed
      return req


   def _build_land(self, speed):
      req = IcarusReq()
      req.type = LAND
      if speed:
         req.speed = speed
      return req


   def _build_move(self, pos, glob, rel, speed, block):
      req = IcarusReq()
      req.type = MOVE
      if pos[0] != None:
         req.move_data.p0 = pos[0]
      if pos[1] != None:
         req.move_data.p1 = pos[1]
      if pos[2] != None:
         req.move_data.p2 = pos[2]
      if speed != None:
         req.speed = speed
      if rel != None:
         req.rel = rel
      if glob != None:
         req.glob = glob
      return req


   def _build_rotate(self, pos, glob, rel, speed, block):
      req = IcarusReq()
      req.type = ROT
      req.pos.extend(pos)
      if speed:
         req.speed = speed
      if rel:
         req.rel = rel
      return req


   def _build_stop(self):
      req = IcarusReq()
      req.type = STOP
      return req



# CARUS client interface
# sends ICARUS commands and reads status


class ICARUS_ClientError(Exception):

   def __init__(self, status, err_msg):
      self.status = status
      self.err_msg = err_msg

   def __str__(self):
      err_map = {E_SYNTAX: 'E_SYNTAX', E_SEMANTIC: 'E_SEMANTIC', E_HARDWARE: 'E_HARDWARE'}
      return 'class: ' + err_map[self.status] + ' message: ' + self.err_msg


class ICARUS_Client:

   def __init__(self, socket):
      self._socket = socket

   def execute(self, req):
      rep = IcarusRep()
      req_data = req.SerializeToString()
      self._socket.send(req_data)
      rep_data = self._socket.recv()
      rep.ParseFromString(rep_data)
      if rep.status != 0:
         raise ICARUS_ClientError(rep.status, rep.message)
      return rep




class StateEventMap(Thread):

   '''
   reads state updates and
   publishes them using the "events" dictionary.
   clients can use emitter.event[name].clear/wait in order
   to wait for an event
   '''

   def __init__(self, socket):
      Thread.__init__(self)
      self._socket = socket
      self.daemon = True
      self.events = {}
      states = ['standing', 'stopping', 'taking_off', 'moving', 'hovering', 'landing']
      for state in states:
         self.events[state] = Event()

   def run(self):
      while True:
         try:
            state = self._socket.recv()
            self.events[state].set()
         except:
            sleep(1)



class ICARUS_SynClient:

   '''
   synchronous icarus command interface client
   sends commands to icarus and waits until completion
   '''

   def __init__(self, ctrl_socket, state_socket):
      self.interface = ICARUS_Client(ctrl_socket)
      self.map = StateEventMap(state_socket)
      self.map.start()

   def execute(self, req):
      type = req.type
      if type == TAKEOFF:
         self.map.events['hovering'].clear()
         self.interface.execute(req)
         self.map.events['hovering'].wait()
      elif type == LAND:
         self.map.events['standing'].clear()
         self.interface.execute(req)
         self.map.events['standing'].wait()
      elif type == MOVE:
         self.map.events['hovering'].clear()
         self.interface.execute(req)
         self.map.events['hovering'].wait()
      elif type == ROT:
         self.interface.execute(req)
      else:
         print 'unknown req type', type

