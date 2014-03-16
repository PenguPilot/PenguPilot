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
 
 ICARUS State Emitter
 - translates flight_sm states to icarus StateUpdate messages
 - sends icarus StateUpdate messages using the given socket

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from icarus_pb2 import StateUpdate


class StateEmitter:

   def __init__(self, socket):
      self._socket = socket

   def send(self, state):
      sm = StateUpdate()
      sm.state = {'standing': STANDING,
                  'taking_off': TAKING_OFF,
                  'hovering': HOVERING,
                  'landing': LANDING,
                  'moving': MOVING,
                  'stopping': STOPPING}[state]
      self._socket.send(sm.SerializeToString())

