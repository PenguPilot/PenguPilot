
class FlightSM:

   def __init__(self, error, entry, takeoff, land, move, stop):
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
                  'done': 'hovering'
               }
            },
            'hovering': {
               'map': {
                  'move': ('moving', move),
                  'land': ('landing', land)
               }
            },
            'landing': {
               'map': {
                  'done': 'standing'
               }
            },
            'moving': {
               'map': {'done': 'hovering',
                       'move': ('moving', move),
                       'stop': ('stopping', stop)
               }
            },
            'stopping': {
               'map': {'done': 'hovering'}
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
         exit_func()
      self.state = new_state
      self.entry(self.state)
      return True

