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
  
 Flight State Machine

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """



class FlightSM:

   def __init__(self, error, entry, takeoff, land, move, stop, kill):
      self.error = error
      self.entry = entry
      self.state = 'standing'
      self.fsm = \
      {
         'map': {
            'standing': {
               'map': {
                  'takeoff': ('taking_off', takeoff)
               }
            },
            'taking_off': {
               'map': {
                  'cancel': 'landing',
                  'done': 'hovering',
                  'kill': ('killing', kill)
               }
            },
            'hovering': {
               'map': {
                  'move': ('moving', move),
                  'land': ('landing', land),
                  'kill': ('killing', kill)
               }
            },
            'landing': {
               'map': {
                  'done': 'standing',
                  'kill': ('killing', kill)
               }
            },
            'moving': {
               'map': {'done': 'hovering',
                       'move': ('moving', move),
                       'stop': ('stopping', stop)
                       'kill': ('killing', kill)
               }
            },
            'stopping': {
               'map': {'done': 'hovering',
               'kill': ('killing', kill)
              }
            },
            'killing' : {
               'map': {'done': 'standing'}
            }
         }
      }

   def handle(self, event):
      try:
         new_state = self.fsm['map'][self.state]['map'][event]
      except KeyError:
         self.error(self.state, event)
         return False
      if len(new_state) == 2:
         new_state, exit_func = new_state
         self.state = new_state
         self.entry(self.state)
         return exit_func()
         
      self.state = new_state
      self.entry(self.state)
      return True

