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
import distutils.sysconfig

re_cc = re.compile('.*\.(c|cpp)$')
re_pb = re.compile('.*\.proto$')

def set_compiler_dependent_cflags():
   cflags = '-D_GNU_SOURCE -pipe -fomit-frame-pointer -std=c99 -O3 -Wall -Wextra '
   command = "cat /proc/cpuinfo"
   all_info = file('/proc/cpuinfo').read()
   board = 'None'
   for line in all_info.split('\n'):
      if "Hardware" in line:
         board = re.sub( ".*Hardware.*: ", "", line, 1)
   print 'scons: Optimization for board: ' + board
   if board == 'ODROID-U2/U3':
      cflags += ' -O3 -pipe -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -ffast-math'
   elif board == 'Gumstix Overo':
      cflags += ' -O3 -ftree-vectorize -ffast-math -march=armv7-a -mtune=cortex-a8 -mfpu=vfpv3-d16 -mfloat-abi=hard'
   elif board == 'BCM2708':
	   cflags += ' -O3 -ftree-vectorize -ffast-math -mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard'
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


# SCL:
scl_bindings_dir = 'scl/shared'
scl_lib = env.Library('scl/shared/scl', collect_files(scl_bindings_dir, re_cc))
append_inc_lib(scl_bindings_dir)

# Shared:
shared_dir = 'shared/'
shared_lib = env.Library(shared_dir + 'shared', collect_files(shared_dir, re_cc))
shared_sh_lib = env.SharedLibrary(shared_dir + 'shared', collect_files(shared_dir, re_cc))
append_inc_lib(shared_dir)

# OPCD:
opcd_pb_dir = 'opcd/shared/'
opcd_pb_lib = make_proto_lib(opcd_pb_dir, 'opcd_pb')
opcd_lib = env.Library('opcd/shared/opcd', collect_files('opcd', re_cc))
Requires(opcd_lib, scl_lib + opcd_pb_lib)
append_inc_lib('opcd/shared')

common_libs = opcd_lib + opcd_pb_lib + scl_lib + shared_lib

# GPS Publisher:
append_inc_lib('gpsp/service/nmealib')
gpsp_dir = 'gpsp/'
gpsp_pb_dir = gpsp_dir + 'shared/'
gpsp_pb_lib = make_proto_lib(gpsp_pb_dir, 'gpsp_pb')
append_inc_lib('gpsp/shared')
gpsp_bin = env.Program('gpsp/service/gpsp', collect_files(gpsp_dir + 'service', re_cc), LIBS = common_libs + gpsp_pb_lib + ['m', 'pthread', 'yaml', 'zmq', 'glib-2.0', 'protobuf-c'])

# Arduino RC / Power Publisher:
arduino_dir = 'arduino/'
arduino_bin = env.Program('arduino/service/arduino', collect_files(arduino_dir + 'service', re_cc), LIBS = common_libs + ['m', 'msgpack', 'pthread', 'yaml', 'zmq', 'glib-2.0', 'protobuf-c'])
Requires(arduino_bin, common_libs)

# Autopilot:
ap_dir = 'autopilot/'
ap_pb_dir = ap_dir + 'shared/'
ap_src = collect_files(ap_dir + 'service', re_cc)
ap_pb_lib = make_proto_lib(ap_pb_dir, 'autopilot_pb')
ap_bin = env.Program(ap_dir + 'service/autopilot', ap_src, LIBS = common_libs + [gpsp_pb_lib, ap_pb_lib] + ['m', 'msgpack', 'pthread', 'yaml', 'zmq', 'glib-2.0', 'protobuf-c'])

# Display:
display_src = map(lambda x: 'display/shared/' + x, ['pyssd1306.c', 'pyssd1306.i', 'ssd1306.c']) + ['shared/i2c/i2c.c']
env.SharedLibrary('display/shared/_pyssd1306.so', display_src)

# build remote:
remote_dir = 'remote/'
remote_src = collect_files(remote_dir + 'service', re_cc)
remote_bin = env.Program(remote_dir + 'service/remote', remote_src, LIBS = ['m', 'opcd', 'opcd_pb', 'pthread', 'shared', 'scl', 'protobuf-c', 'yaml', 'zmq', 'glib-2.0'])
Requires(remote_bin, common_libs)

# HLFM:
hlfm_dir = 'hlfm/'
hlfm_pb_dir = hlfm_dir + 'shared/'
hlfm_pb_lib = make_proto_lib(hlfm_pb_dir, 'hlfm_pb')

