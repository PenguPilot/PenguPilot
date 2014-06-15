#!/usr/bin/env python

import atexit
import os
import readline
import rlcompleter

from scl import generate_map
from icarus_interface import ICARUS_SynClient, ICARUS_MissionFactory
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
_map = generate_map('mission')
_client = ICARUS_SynClient(_map['icarus_ctrl'], _map['icarus_state'])
i = ICARUS_MissionFactory()

def request(item):
   try:
      _client.execute(item)
   except Exception, e:
      print e

