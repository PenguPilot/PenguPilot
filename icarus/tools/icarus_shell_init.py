#!/usr/bin/env python

import atexit
import os
import readline
import rlcompleter

from scl import generate_map
from icarus_interface import ICARUS_Client, ICARUS_MissionFactory
from misc import user_data_dir


# set-up command history:
_path = user_data_dir + os.sep + 'ICARUS_shell.history'
_history = os.path.expanduser(_path)
def _save_history(historyPath = _history):
   readline.write_history_file(_history)
if os.path.exists(_history):
   readline.read_history_file(_history)
readline.parse_and_bind("tab: complete")
atexit.register(_save_history)


# define
_socket = generate_map('icarus_shell')['ctrl']
_client = ICARUS_Client(_socket)
i = ICARUS_MissionFactory()

def request(item):
   try:
      _client.execute(item)
   except Exception, e:
      print e

