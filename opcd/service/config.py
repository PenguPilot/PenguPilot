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
  
 OPCD Config File Handling

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import yaml
import copy
import os
from misc import user_data_dir


class ConfigError(Exception):

   def __init__(self, msg):
      assert isinstance(msg, str)
      self.msg = msg

   def __str__(self):
      return self.msg



class Config:

   def __init__(self):
      self.LEAF_TYPES = [str, int, float, bool]
      tail = os.sep + 'config' + os.sep + 'params.yaml'
      self.base_path = os.getenv('PENGUPILOT_PATH') + tail
      self.overlay_path = user_data_dir + tail
      print self.overlay_path
      # load base config and overlay of present:
      self.base = yaml.load(file(self.base_path))
      try:
         self.overlay = yaml.load(file(self.overlay_path))
      except:
         self.overlay = {}
      # check single document integrity:
      for doc in self.base, self.overlay:
         self._check_tree(doc)
      # check inter-document integrity:
      for key in self.get_all_keys(self.overlay):
         self._validate_overlay_key(key)


   def set(self, key, val):
      '''
      set attribute identified by key to val
      '''
      self._validate_overlay_key_against(key, val)
      if self._find_entry_or_none(self.base, key) == None:
         raise ConfigError('cannot override unknown attribute "' + key + '"')
      self._insert_val(self.overlay, key, val)


   def get(self, key):
      '''
      get attribute using key
      '''
      try:
         return self._find_entry(self.overlay, key)
      except KeyError:
         try:
            return self._find_entry(self.base, key)
         except KeyError:
            raise ConfigError(key + ' was not found in base config')


   def persist(self): 
      '''
      write configuration overlay to filesystem
      '''
      if len(self.overlay) == 0:
         try:
            os.unlink(self.overlay_path)
         except:
            pass
      else:
         dump = '\n#\n# GENERATED FILE - DO NOT EDIT!\n#\n\n'
         dump += yaml.safe_dump(self.overlay, indent = 3, default_flow_style = False)
         dump += '\n'
         overlay_file = file(self.overlay_path, 'w')
         overlay_file.write(dump)
         overlay_file.close()


   def _check_tree(self, node):
      if isinstance(node, dict):
         for key, node in node.iteritems():
            if isinstance(key, str) and '.' in key:
               raise ConfigError('key ' + str(key) + ' must not contain a dot (.) character')
            self._check_tree(node)
      else:
         if node.__class__ not in self.LEAF_TYPES:
            raise ConfigError('node ' + str(node) + ' must be one of: ' + str(self.LEAF_TYPES))


   def _insert_val(self, node, key, val):
      if isinstance(node, dict):
         head, tail = self._split_key(key)
         if not tail:
            node[head] = val
         else:
            if head not in node.keys():
               node[head] = {}
            node = node[head]
            self._insert_val(node, tail, val)


   def get_all_keys(self, node):
      if isinstance(node, dict):
         list = []
         for key, node in node.iteritems():
            if not isinstance(node, dict):
               list.append(key)
            else:
               sub_list = self.get_all_keys(node)
               list.extend(map(lambda x : key + '.' + x, sub_list))
         return list


   def _find_entry_or_none(self, node, key):
      try:
         return self._find_entry(node, key)
      except KeyError:
         return


   def _split_key(self, key):
      if '.' in key:
         pos = key.find('.')
         head = key[0 : pos]
         tail = key[pos + 1 : ]
      else:
         head = key
         tail = None
      return head, tail

   
   def _validate_overlay_key_against(self, key, val):
      cls = val.__class__
      try:
         base_class = self._find_entry(self.base, key).__class__
      except AssertionError:
         raise
      except:
         raise ConfigError('overlay defines key "' + key + '", which does not exist in base')
      if cls != base_class:
         raise ConfigError('different data types for overlay (' + str(cls) + ') and base (' + str(base_class) + ') for key: ' + key)



   def _validate_overlay_key(self, key):
      overlay_class = self._find_entry(self.overlay, key).__class__
      try:
         base_class = self._find_entry(self.base, key).__class__
      except AssertionError:
         raise
      except:
         raise ConfigError('overlay defines key "' + key + '", which does not exist in base')
      if overlay_class != base_class:
         raise ConfigError('different data types for overlay (' + str(overlay_class) + ') and base (' + str(base_class) + ') for key: ' + key)


   def _find_entry(self, node, key):
      if isinstance(node, dict):
         try:
            head, tail = self._split_key(key)
         except TypeError:
            return node
         node = node[head]
         return self._find_entry(node, tail)
      else:
         assert node.__class__ in self.LEAF_TYPES
         return node

