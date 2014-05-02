"""
  __________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|

 SCL Specification Generator
 converts system specification into component socket specification,
 enriched with the corresponding zeromq socket-pair definitions

 REQ <-> REP: 1 shared link
 PUB --> SUB: 1 publisher, multiple subscribers

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import sys
import yaml
import zmq
import argparse
import re


class Regex:

   def __init__(self, re_str):
      self.regex = re.compile(re_str)
      self.re_str = re_str

   def matching(self, check_str):
      try:
         return self.regex.match(check_str)
      except:
         print 'oops'
         return False


def parse_args():
   parser = argparse.ArgumentParser(description = 'ZMQ_IPC specification code generator.')
   parser.add_argument('--debug', type = str, metavar = 'debug', default = 'False', help = "True (TCP/IP) or False (POSIX IPC)")
   parser.add_argument('--base_id', type = int, metavar = 'base_id', default = 10000, help = "base id for socket (port) or IPC (prefix_id)")
   return parser.parse_args()


try:
   socket_type_map = {'sub': zmq.SUB, 'pub': zmq.PUB, 'req': zmq.REQ, 'rep': zmq.REP}
   # regex definitions:
   ident_re = Regex('^[a-zA-Z_][a-zA-Z_0-9]*$') # identifier
   comp_socket_re = Regex('^[a-zA-Z_][a-zA-Z_0-9]*[.][a-zA-Z_][a-zA-Z_0-9]*$') # component.socket (2 identifiers)

   # parse command line arguments:
   args = parse_args()

   # load yaml file from stdin:
   components = yaml.load(sys.stdin)
   if not isinstance(components, list):
      raise AssertionError('top level type of config file must be list, got: %s' % data.__class__)

   # comp_map maps (comp, socket) tuples to socket_type
   comp_map = {}
   socket_map = {}
   comp_count = 0
   socket_id = 0
   if not isinstance(components, list):
      raise AssertionError('components must be a list, got: %s' % components.__class__)
   if len(components) == 0:
      raise AssertionError("components list must not be empty")
   for comp in components:
      if not isinstance(comp, dict):
         raise AssertionError("component %d must be a dict, got: %s" % comp.__class__)
      try:
         comp_name = comp['name']
      except:
         raise AssertionError('component %d must contain a name field' % comp_count)
      if not ident_re.matching(comp_name):
         raise AssertionError("component %d's name field did not match regex: %s" % (comp_count, ident_re.re_str))
      try:
         sockets = comp['sockets']
      except:
         raise AssertionError('component %s (%d) must contain a sockets field' % (comp_name, comp_count))
      if not isinstance(sockets, list):
         raise AssertionError("socket structure of component %s (%d) must be a list, got: %s" % (comp_name, comp_count, comp.__class__))
      if len(sockets) == 0:
         raise AssertionError("socket list of component %s (%d) must not be empty" % (comp_name, comp_count))
      socket_count = 1
      for socket in sockets:
         if not isinstance(socket, dict):
            raise AssertionError("socket %d [component %s (%d)] must be a dict, got: %s" % (socket_count, comp_name, comp_count, socket.__class__))
         if len(socket) != 1:
            raise AssertionError("length of socket %d [component %s (%d)] must be 1" % (socket_count, comp_name, comp_count))
         socket_name, socket_type = socket.items()[0]
         if not ident_re.matching(socket_name):
            raise AssertionError("socket %d's name [component %s (%d)] did not match regex: %s" % (comp_count, ident_re.re_str))
         if not socket_type in socket_type_map.keys():
            raise AssertionError("socket %d's type [component %s (%d)] was not found in: %s" % socket_type_map.keys())
         comp_map[(comp_name, socket_name)] = socket_type
         if args.debug == 'True':
            socket_str = 'tcp://localhost:%d' % (args.base_id + socket_id)
         else:
            socket_str = 'ipc:///tmp/scl_%d' % (args.base_id + socket_id)
         if socket_name not in socket_map.keys():
            socket_map[socket_name] = socket_str
            socket_id += 1
      comp_count += 1

   zmq_spec = {}
   for (comp_name, socket_name), socket_type in comp_map.items():
      if not comp_name in zmq_spec:
         zmq_spec[comp_name] = []
      entry = {'socket_name': socket_name,
               'zmq_socket_type': socket_type_map[socket_type],
               'zmq_socket_path': socket_map[socket_name]}
      zmq_spec[comp_name].append(entry)

   sys.stdout.write(yaml.dump(zmq_spec))

except Exception, e:
   sys.stderr.write(str(e) + '\n')
   sys.exit(1)

