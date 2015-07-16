from scl import scl_get_socket

ctrl = scl_get_socket('ap_ctrl', 'req')
state = scl_get_socket('ap_state', 'sub')

cmds = [
[
'takeoff',
'move', 'spr', 4.0, 1.0],
'land'
]

for cmd in cmds:
   ctrl.send(cmd)
   rep = ctrl.recv()
   if not rep[0]:
      print rep[1]
   while state.recv() not in ['hovering', 'standing']:
       pass
