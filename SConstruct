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
  
 scons build script

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import os
import re
import subprocess


re_cc = re.compile('.*\.(c|cpp)$')
re_pb = re.compile('.*\.proto$')


def set_compiler_dependent_cflags():
   cflags = '-pipe -std=gnu99 -Wall -Wextra -O3 -g '
   pipe = subprocess.Popen([env['CC'], '-v'], env=env['ENV'], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   if 'armv7a' in pipe.stderr.read():
      cflags += ' -march=armv7-a -mtune=cortex-a8 -mfpu=neon -ftree-vectorize -mfpu=neon -mfloat-abi=softfp -ffast-math -fomit-frame-pointer -funroll-loops'
   env['CFLAGS'] = cflags


def collect_files(path, regex):
   flist = []
   for root, sub_folders, files in os.walk(path):
      for file in files:
         f = os.path.join(root, file)
         if regex.match(f):
            flist.append(f)
   return flist


def append_inc_lib(path):
   env['CPPPATH'] += [path]
   env['LIBPATH'] += [path]


def make_proto_lib(path, name):
   pb_c = []
   for proto in collect_files(path, re_pb):
      env.Protoc([], proto)
      pb_c += env.ProtocC([], proto)
   append_inc_lib(path)
   return env.Library(path + name, pb_c)


env = Environment(
    ENV = {'PATH' : os.environ['PATH'],
           'TERM' : os.environ['TERM'],
           'HOME' : os.environ['HOME']},
    tools=['default', 'protoc', 'protoc-c'])


set_compiler_dependent_cflags()

env.ParseConfig('pkg-config --cflags glib-2.0')
env['CPPPATH'] += ['.', 'shared', 'tools']
env['LIBPATH'] = ['shared']


# build scl:
scl_bindings_dir = 'scl/shared'
scl_lib = env.Library('scl/shared/scl', collect_files(scl_bindings_dir, re_cc))
append_inc_lib(scl_bindings_dir)

# build shared:
shared_dir = 'shared/'
shared_lib = env.Library(shared_dir + 'shared', collect_files(shared_dir, re_cc))
append_inc_lib(shared_dir)

# build opcd:
opcd_pb_dir = 'opcd/shared/'
opcd_pb_lib = make_proto_lib(opcd_pb_dir, 'opcd_pb')
opcd_lib = env.Library('opcd/shared/opcd', collect_files('opcd', re_cc))
Requires(opcd_lib, scl_lib + opcd_pb_lib)
append_inc_lib('opcd/shared')

# build gps:
gps_dir = 'sensors/gps/'
gps_pb_dir = gps_dir + 'shared/'
gps_pb_lib = make_proto_lib(gps_pb_dir, 'gps_pb')
gps_bin = env.Program('sensors/gps/service/gps', collect_files(gps_dir + 'service', re_cc), LIBS = ['pthread', 'opcd', 'opcd_pb', 'shared', 'scl', 'yaml-cpp', 'zmq', 'glib-2.0', 'gps_pb', 'protobuf-c'])
Requires(gps_bin, scl_lib + shared_lib + gps_pb_lib + opcd_pb_lib)

# build powerman:
pm_pb_lib = make_proto_lib('powerman/shared/', 'powerman_pb')

# build rc:
rc_dir = 'rc/'
rc_pb_dir = rc_dir + 'shared/'
rc_src = collect_files(rc_dir + 'service', re_cc)
rc_pb_lib = make_proto_lib(rc_pb_dir, 'rc_pb')
rc_bin = env.Program(rc_dir + 'service/rc', rc_src, LIBS = ['opcd', 'opcd_pb', 'shared', 'scl', 'protobuf-c', 'rc_pb', 'yaml-cpp', 'zmq', 'glib-2.0'])
Requires(rc_bin, scl_lib + shared_lib + opcd_lib + opcd_pb_lib + rc_pb_lib)

# build autopilot:
ap_dir = 'autopilot/'
ap_pb_dir = ap_dir + 'shared/'
ap_src = collect_files(ap_dir + 'service', re_cc)
ap_pb_lib = make_proto_lib(ap_pb_dir, 'autopilot_pb')
ap_bin = env.Program(ap_dir + 'service/autopilot', ap_src, LIBS = ['m', 'msgpack', 'meschach', 'pthread', 'opcd', 'opcd_pb', 'shared', 'scl', 'powerman_pb', 'gps_pb', 'autopilot_pb', 'protobuf-c', 'yaml-cpp', 'zmq', 'glib-2.0'])
Requires(ap_bin, pm_pb_lib + scl_lib + opcd_lib + opcd_pb_lib + ap_pb_lib)

# build icarus:
ic_dir = 'icarus/'
ic_pb_dir = ic_dir + 'shared/'
ic_pb_lib = make_proto_lib(ic_pb_dir, 'icarus_pb')
