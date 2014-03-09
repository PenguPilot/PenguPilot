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
 
 State Translation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from flight_sm import flight_Standing, flight_Stopping, flight_Taking_off, flight_Landing, flight_Moving, flight_Hovering
from icarus_pb2 import STANDING, TAKING_OFF, STOPPING, LANDING, MOVING, HOVERING


_flight_state_map = {flight_Standing:   'STANDING',
                     flight_Stopping:   'STOPPING',
                     flight_Taking_off: 'TAKING_OFF',
                     flight_Moving:     'MOVING',
                     flight_Hovering:   'HOVERING',
                     flight_Landing:    'LANDING'}


def to_string(state):
   return _flight_state_map[state.__class__]


def to_protocol(state):
   return globals()[to_string(state)]


