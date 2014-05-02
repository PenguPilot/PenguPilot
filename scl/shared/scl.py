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

 SCL Python interface
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


import os, yaml, zmq, subprocess


class ZSEx(Exception):
   """
   zeromq specification exception
   """

   pass


def generate_map(component_name):

   """
   generates zeromq socket map for a certain component
   """

   # figure out common base path:
   try:
      cache = os.environ["HOME"] + "/.SCL/scl.yaml"
   except:
      raise ZSEx("HOME environment variable is not set")
   try:
      generator = os.environ["SCL_CACHE_GENERATOR"]
   except:
      raise ZSEx("SCL_CACHE_GENERATOR environment variable is not set")


   # call code generator:
   try:
      result = subprocess.call(["bash", generator])
      if result != 0:
         raise ZSEx("%s failed with code %d" % (generator, result))
   except:
      raise ZSEx("could not execute %s" % generator)

   # load generated zmq specification:
   try:
      zmq_spec = yaml.load(file(cache))
   except:
      raise ZSEx("could not load " + zmq_spec_path)
   
   # generate the map:
   context = zmq.Context()
   socket_map = {}
   for entry in zmq_spec[component_name]:
      socket_name = entry['socket_name']
      socket_type = entry['zmq_socket_type']
      socket_path = entry['zmq_socket_path']
      socket = context.socket(socket_type)
      if socket_type in [zmq.SUB, zmq.REQ]:
         if socket_type == zmq.SUB:
            socket.setsockopt(zmq.SUBSCRIBE, "")
         socket.connect(socket_path)
      elif socket_type in [zmq.PUB, zmq.REP]:
         socket.bind(socket_path)
      else:
         raise ZSEx("unknown socket type: %d" % socket_type)
      socket_map[socket_name] = socket

   return socket_map

