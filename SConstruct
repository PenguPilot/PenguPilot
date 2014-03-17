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
  
 SCons Build Script

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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
import distutils.sysconfig

re_cc = re.compile('.*\.(c|cpp)$')
re_pb = re.compile('.*\.proto$')


def set_compiler_dependent_cflags():
   cflags = '-D_GNU_SOURCE -pipe -std=c99 -Wall -Wextra '
   pipe = subprocess.Popen([env['CC'], '-v'], env=env['ENV'], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   if 'armv7a' in pipe.stderr.read():
      cflags += ' -O3 -ftree-vectorize -ffast-math -fomit-frame-pointer -funroll-loops -march=armv7-a -mtune=cortex-a8 -mfpu=vfpv3-d16 -mfloat-abi=hard'
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
    SWIGFLAGS=['-python'],
    CPPPATH=[distutils.sysconfig.get_python_inc()],
    SHLIBPREFIX="",
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

# build powerman:
pm_pb_lib = make_proto_lib('powerman/shared/', 'powerman_pb')
common_libs = scl_lib + shared_lib + opcd_lib + opcd_pb_lib

# build autopilot:
ap_dir = 'autopilot/'
ap_pb_dir = ap_dir + 'shared/'
ap_src = collect_files(ap_dir + 'service', re_cc)
ap_pb_lib = make_proto_lib(ap_pb_dir, 'autopilot_pb')
ap_bin = env.Program(ap_dir + 'service/autopilot', ap_src, LIBS = ['m', 'msgpack', 'meschach', 'pthread', 'opcd', 'opcd_pb', 'shared', 'scl', 'powerman_pb', 'gps_pb', 'autopilot_pb', 'protobuf-c', 'yaml', 'zmq', 'glib-2.0'])
Requires(ap_bin, common_libs + pm_pb_lib + ap_pb_lib)

# build gps publisher:
append_inc_lib('gps/shared')
append_inc_lib('gps/service/nmealib')
gps_dir = 'gps/'
gps_pb_dir = gps_dir + 'shared/'
gps_pb_lib = make_proto_lib(gps_pb_dir, 'gps_pb')
gps_bin = env.Program('gps/service/gps', collect_files(gps_dir + 'service', re_cc), LIBS = ['m', 'pthread', 'opcd', 'opcd_pb', 'shared', 'scl', 'yaml', 'zmq', 'glib-2.0', 'gps_pb', 'protobuf-c'])
Requires(gps_bin, common_libs + gps_pb_lib)

# build display:
display_src = map(lambda x: 'display/shared/' + x, ['pyssd1306.c', 'pyssd1306.i', 'i2c/i2c.c', 'ssd1306.c'])
env.SharedLibrary('display/shared/_pyssd1306.so', display_src)

# build icarus:
ic_dir = 'icarus/'
ic_pb_dir = ic_dir + 'shared/'
ic_pb_lib = make_proto_lib(ic_pb_dir, 'icarus_pb')
