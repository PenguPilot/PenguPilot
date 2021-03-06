
from scl import scl_get_socket
import inspect


LL_ERROR =   0 
LL_WARNING = 1
LL_INFO =    2
LL_DEBUG =   3


def logger_init(name):
   global _name, _socket
   _name = name
   _socket = scl_get_socket('log_data', 'push')


def log(level, msg):
   frame = inspect.stack()[1][0]
   info = inspect.getframeinfo(frame)
   _socket.send([_name, level, info.filename, int(info.lineno), msg])

