#!/usr/bin/env python
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
  
 Service Control Utility

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


def parse_args():
   parser = argparse.ArgumentParser(description = 'svctrl - service control utility')
   group = parser.add_mutually_exclusive_group()
   group.add_argument('--start', type = str, metavar = 'name')
   group.add_argument('--stop', type = str, metavar = 'name')
   group.add_argument('--restart', type = str, metavar = 'name')
   args = parser.parse_args()
   if args.start:
      return 'start', args.start
   elif args.stop:
      return 'stop', args.stop
   elif args.restart:
      return 'restart', args.restart
   else:
      return 'show'



def read_config():

   class Regex:

      def __init__(self, re_str):
         self.regex = re.compile(re_str)
         self.re_str = re_str

      def matching(self, check_str):
         try:
            return self.regex.match(check_str)
         except:
            return False

   try:
      # regex definition for identifier:
      ident_re = Regex('^[a-zA-Z_][a-zA-Z_0-9]*$')

      # load yaml file:
      comp_base = os.getenv('PENGUPILOT_PATH')
      conf_file = comp_base + os.sep + 'config' + os.sep + 'services.yaml'
      services = yaml.load(file(conf_file))

      # assert that we have a dict of services:
      if not isinstance(services, dict):
         raise AssertionError('top level type of config file must be dict, got: %s' % data.__class__)

      count = 1
      config = {}
      for name, service in services.items():
         #if not ident_re.matches(name):
         #   raise AssertionError('service %d name is not a valid identifier' % count)
         if not isinstance(service, dict):
            raise AssertionError("service %d data must be a dict, got: %s" % service.__class__)
         
         # get service binary:
         binary = comp_base + os.sep + os.path.expandvars(service['binary'])

         # retrieve and check arguments:
         try:
            args = os.path.expandvars(service['args'])
            cmd = binary + ' ' + args
         except:
            cmd = binary

         # retrieve and check service dependencies:
         try:
            depends = service['depends']
            if not isinstance(depends, list):
               raise AssertionError('service %s must contain a name and a binary field' % count)
            for depend in depends:
               if not depend in services:
                  raise AssertionError('service %s dependency %s unknown' % (name, depend))
         except:
            depends = []
         # insert the service:
         config[name] = (cmd, depends)
         count += 1

      # return services config map
      return config

   except Exception, e:
      sys.stderr.write(str(e) + '\n')
      sys.exit(1)


def running_processes():
   processes = []
   for name in config.keys():
      if validate(name):
         processes.append(name)
   return processes


def parent_deps(name):
   l = []
   for item in config.items():
      if name in item[1][1]:
         l.append(item[0])
   return l


def calc_reverse_deps(name):
   order = []
   queue = [name]
   order.append(name)
   while len(queue) > 0:
      _name = queue[0]
      for dep in parent_deps(_name):
         if not dep in queue:
            order.append(dep)
            queue.append(dep)
      queue = queue[1:]
   order.reverse()
   return order


def toposort():
   """ stolen from: http://code.activestate.com/recipes/578272-topological-sort """
   _data = config
   data = {}
   for item in _data.items():
      data[item[0]] = set(item[1][1])
   from functools import reduce
   for k, v in data.items():
      v.discard(k)
   extra_items_in_deps = reduce(set.union, data.itervalues()) - set(data.iterkeys())
   data.update({item:set() for item in extra_items_in_deps})
   while True:
      ordered = set(item for item, dep in data.iteritems() if not dep)
      if not ordered:
         break
      yield ordered
      data = {item: (dep - ordered)
              for item, dep in data.iteritems()
                 if item not in ordered}
   assert not data, "Cyclic dependencies exist among these items:\n%s" % '\n'.join(repr(x) for x in data.iteritems())


def restart_order(name):
   topo = toposort()
   ordered = []
   deps = set(calc_reverse_deps(name))
   for level in [x for x in toposort()]:
      ordered.extend(deps & level)
   return ordered


def calc_deps_reverse_bfs(name):
   list = []
   queue = []
   queue.append(name)
   name_count = 0
   while len(queue) > 0:
      _name = queue[0]
      if _name == name:
         name_count += 1
         if name_count > 1:
            raise AssertionError('detected cyclic dependency for service %s' % name)
      queue = queue[1:]
      if not _name in list:
         list.append(_name)
      queue.extend(config[_name][1])    
   list.reverse()
   return list


from termcolor import red, blue, green
try:
   import sys
   import yaml
   import re, os
   import argparse
   from processes import start, stop, validate
   args = parse_args()
   config = read_config()
   if args == 'show':
      # show list and status of services
      max_name_len = 0
      for name in config:
         name_len = len(name)
         if name_len > max_name_len:
            max_name_len = name_len
      for name, (path, _) in config.items():
         skip = ' ' * (max_name_len - len(name))
         pid = validate(name)
         if pid:
            ex_str = green('running [%d]' % pid)
         else:
            ex_str = red('not running')
         try:
            if len(config[name][1]) != 0:
               ex_str += ', depends: ' + str(config[name][1])
         except:
            pass
         print '%s:%s %s' % (blue(name), skip, ex_str)

   elif args[0] in ['start', 'stop']:
      try:
         name = args[1]
         data = config[name]
         if args[0] == 'start':
            names = calc_deps_reverse_bfs(name)
            if len(names) > 1:
               print 'dependency resolution order:', names
            for name in names:
               start(name, config[name][0])
         else:
            stop(name)
      except KeyError, e:
         print red('ERROR:'), 'service %s is unknown' % args[1]
      except Exception, e:
         print red('ERROR:'), 'service %s failed to start/stop: %s' % (args[1], str(e))

   elif args[0] == 'restart':
      import copy
      name = args[1]
      names = []
      running = running_processes()
      for service in restart_order(name):
         if service in running:
            names.append(service)
      rev_names = copy.deepcopy(names)
      rev_names.reverse()
      if len(rev_names) > 1:
         print 'stop resolution order:', rev_names
      for name in rev_names:
         stop(name)
      if len(names) > 1:
         print 'start resolution order:', names
      for name in names:
         start(name, config[name][0])

except KeyboardInterrupt:
   print red('NOTE:'), 'operation canceled by user'

