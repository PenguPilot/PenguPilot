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
 converts system specification into component gate specification,
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
   comp_gate_re = Regex('^[a-zA-Z_][a-zA-Z_0-9]*[.][a-zA-Z_][a-zA-Z_0-9]*$') # component.gate (2 identifiers)

   # parse command line arguments:
   args = parse_args()

   # load yaml file from stdin:
   data = yaml.load(sys.stdin)
   if not isinstance(data, dict):
      raise AssertionError('top level type of config file must be dict, got: %s' % data.__class__)
   if len(data) != 2:
      raise AssertionError('top level dict must contain 2 entries, got: %d' % len(data))

   # comp_map maps (comp, gate) tuples to gate_type
   comp_map = {}
   comp_count = 1
   expected_keys = set(['components', 'connections'])
   if set(data.keys()) != expected_keys:
      raise AssertionError('top level dict must contain keys: %s' % expected_keys)
   components = data['components']
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
         comp_gates = comp['gates']
      except:
         raise AssertionError('component %s (%d) must contain a gates field' % (comp_name, comp_count))
      if not isinstance(comp_gates, list):
         raise AssertionError("gate structure of component %s (%d) must be a list, got: %s" % (comp_name, comp_count, comp.__class__))
      if len(comp_gates) == 0:
         raise AssertionError("gate list of component %s (%d) must not be empty" % (comp_name, comp_count))
      gate_count = 1
      for gate in comp_gates:
         if not isinstance(gate, dict):
            raise AssertionError("gate %d [component %s (%d)] must be a dict, got: %s" % (gate_count, comp_name, comp_count, gate.__class__))
         if len(gate) != 1:
            raise AssertionError("length of gate %d [component %s (%d)] must be 1" % (gate_count, comp_name, comp_count))
         gate_name, gate_type = gate.items()[0]
         if not ident_re.matching(gate_name):
            raise AssertionError("gate %d's name [component %s (%d)] did not match regex: %s" % (comp_count, ident_re.re_str))
         if not gate_type in socket_type_map.keys():
            raise AssertionError("gate %d's type [component %s (%d)] was not found in: %s" % socket_type_map.keys())
         comp_map[(comp_name, gate_name)] = gate_type
         gate_count += 1
      comp_count += 1

   # conns is a list containing: ((comp1, gate1, type1), (comp2, gate2, type2))
   conns = []
   comp_gates_seen = set()
   connections = data['connections']
   if not isinstance(connections, list):
      raise AssertionError('connections must be a list, got: %s' % connections.__class__)
   if len(connections) == 0:
      raise AssertionError('at least one connection needs to be defined')
   conn_count = 1
   for conn in connections:
      if not isinstance(conn, list):
         raise AssertionError('connection %d must be a list, got: %s' % (conn_count, conn.__class))
      if len(conn) != 2:
         raise AssertionError('connection %d must contain 2 component.gate entries')
      comp_gates = []
      for i in range(2):
         conn_entry = conn[i]
         if not comp_gate_re.matching(conn_entry):
            raise AssertionError('connection %d, index %d did not match regex: %s' % (conn_count, i, comp_gate_re.re_str))
         comp_gates.append(conn_entry.split('.'))
      types = []
      for i in range(2):
         comp, gate = comp_gates[i]
         types.append(comp_map[(comp, gate)])
         if (comp, gate) not in comp_map.keys():
            raise AssertionError('connection %d, index %d is an unknown component/gate combination' % (comp_count, i))
         comp_gates_seen |= set([(comp, gate)])
      valid_types_set = [set(['pub', 'sub']), set(['req', 'rep'])]
      cgt_list = []
      for i in range(2):
         cgt_list.append(tuple(comp_gates[i] + [types[i]]))
         connected_types = set([types[i], types[1 - i]])
         if connected_types not in valid_types_set:
            raise AssertionError('invalid type combination: %s, expected one of: %s' % (connected_types, valid_types_set))

      conns.append(tuple(cgt_list))
      conn_count += 1

   #gates_unconn = set(comp_map.keys()) - comp_gates_seen
   #if len(gates_unconn) > 0:
   #   raise AssertionError('unconnected gates detected: ' % map(lambda x: '%s.%s' % (x[0], x[1]), gates_unconn))

   # create socket generating set of components/gates:
   sock_gen_set = set()
   for conn in conns:
      for i in range(2):
         if conn[i][2] in ['pub', 'rep']:
            sock_gen_set |= set([conn[i][0:2]])
   sock_gen_list = list(sock_gen_set)

   # build socket map for pub & rep:
   socket_map = {}
   for i in range(len(sock_gen_list)):
      if args.debug == 'True':
         socket_str = 'tcp://localhost:%d' % (args.base_id + i)
      else:
         socket_str = 'ipc:///tmp/scl_%d' % (args.base_id + i)
      socket_map[sock_gen_list[i]] = socket_str

   # build socket map for sub & req:
   for conn in conns:
      for i in range(2):
         if conn[i][2] in ['sub', 'req']:
            socket_map[tuple(conn[i][0:2])] = socket_map[tuple(conn[1 - i][0:2])]

   # finally, build zmq specification:
   zmq_spec = {}
   for (comp_name, gate_name), gate_type in comp_map.items():
      if not comp_name in zmq_spec:
         zmq_spec[comp_name] = []
      entry = {'gate_name': gate_name,
               'zmq_socket_type': socket_type_map[gate_type],
               'zmq_socket_path': socket_map[(comp_name, gate_name)]}
      zmq_spec[comp_name].append(entry)

   # write generated code to stdout:
   sys.stdout.write(yaml.dump(zmq_spec))

except Exception, e:
   sys.stderr.write(str(e) + '\n')
   sys.exit(1)

