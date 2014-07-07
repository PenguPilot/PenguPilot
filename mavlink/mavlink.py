
from scl import generate_map
from util.mavio import MAVIO_Serial, MAVIO_UDP
from util.gendisp import GenDisp
from util.mavlink_util import MAVLink_Interface
from handlers.params import ParamHandler
#from handlers.icarus import ICARUS_Handler
from handlers.deadbeef import DeadbeefHandler
#from bridges.core_bridge import CoreBridge
#from bridges.gps_bridge import GpsBridge
from time import sleep
#from arbiter_interface import ArbiterInterface


# get socket map and open mavio:
socket_map = generate_map('mavlink')
mavio = MAVIO_UDP('10.0.0.7', 14550, source_system = 1)
#mavio = MAVIO_Serial('/dev/ttyUSB1', 9600, source_system = 1)
#arbiter_interface = ArbiterInterface(socket_map['arbiter_ctrl'])

# start parameter handlers and dispatcher (handles all incoming data):
dispatcher = GenDisp(mavio)
#handlers = [ParamHandler(dispatcher),
#   ICARUS_Handler(dispatcher, arbiter_interface),
#   DeadbeefHandler(dispatcher)]
#dispatcher.start(handlers)

# start outgoing bridges:
mav_iface = MAVLink_Interface(mavio)
#core_bridge = CoreBridge(socket_map, mav_iface, 0.5, dispatcher)
#gps_bridge = GpsBridge(socket_map, mav_iface, 1.0)

while 1:
   sleep(1)
