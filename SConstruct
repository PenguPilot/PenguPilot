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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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
   cflags = '-O3 -D_GNU_SOURCE -std=c99 -Wall -Wextra -Werror -Wno-strict-aliasing -Wno-unused-variable -Wno-unused-parameter -Wno-unused-function -Wno-unused-but-set-variable -Wno-error=unused-result'
   all_info = file('/proc/cpuinfo').read()
   board = 'None'
   for line in all_info.split('\n'):
      if "Hardware" in line:
         board = re.sub( ".*Hardware.*: ", "", line, 1)
   print 'scons: Optimization for board: ' + board
   if board == 'ODROID-U2/U3':
      cflags += ' -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard'
   elif board == 'Gumstix Overo':
      cflags += ' -march=armv7-a -mtune=cortex-a8 -mfpu=vfpv3-d16 -mfloat-abi=hard'
   elif board == 'BCM2708':
	   cflags += ' -mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard'
   env['CFLAGS'] = cflags
   env['CXXFLAGS'] = cflags + ' -Wno-error'

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

# Logger Library:
logger_dir = 'logger/'
logger_shared_dir = logger_dir + 'shared/'
logger_lib = env.Library(logger_shared_dir + 'logger', collect_files(logger_shared_dir, re_cc))
append_inc_lib(logger_shared_dir)

# OPCD:
opcd_pb_dir = 'opcd/shared/'
opcd_pb_lib = make_proto_lib(opcd_pb_dir, 'opcd_pb')
opcd_lib = env.Library('opcd/shared/opcd', collect_files('opcd', re_cc))
Requires(opcd_lib, scl_lib + opcd_pb_lib)

common_libs = logger_lib + opcd_lib + opcd_pb_lib + scl_lib + shared_lib

def build_service(name, extra_libs = []):
   append_inc_lib(name + '/shared')
   dir = 'service'
   bin = env.Program(name + '/' + dir + '/' + name, collect_files(name + '/' + dir, re_cc), LIBS = extra_libs + common_libs + ['m', 'rt', 'pthread', 'yaml', 'zmq', 'glib-2.0', 'protobuf-c', 'msgpack'])

append_inc_lib('gpsp/service/nmealib')

build_service('gpsp')
build_service('penguio_mw')
build_service('test_service')
build_service('gpstime')
build_service('rc_cal')
build_service('gyro_cal')
build_service('mag_adc_cal')
build_service('gps_rel')
build_service('acc_rot_neu')
build_service('acc_hpf_neu')
build_service('pos_speed_est_neu')
build_service('cmc')
build_service('acc_cal')
build_service('ahrs')
build_service('rs_ctrl')
build_service('rs_ctrl_prx')
build_service('rp_ctrl_prx')
build_service('arduino')
build_service('ofs')
build_service('i2c_sensors')
build_service('mixer_prx')

# Display:
display_src = map(lambda x: 'display/shared/' + x, ['pyssd1306.c', 'pyssd1306.i', 'ssd1306.c']) + ['shared/i2c/i2c.c']
env.SharedLibrary('display/shared/_pyssd1306.so', display_src)

# Serialport:
serialport_src = map(lambda x: 'shared/' + x, ['serial.c', 'serialport.i'])
env.SharedLibrary('shared/_serialport.so', serialport_src)

# Remote Control Service:
# Library:
remote_dir = 'remote/'
remote_shared_dir = remote_dir + 'shared/'
remote_lib = env.Library(remote_shared_dir + 'remote', collect_files(remote_shared_dir, re_cc), LIBS = common_libs + ['m', 'remote', 'opcd', 'opcd_pb', 'pthread', 'shared', 'scl', 'protobuf-c', 'yaml', 'zmq', 'glib-2.0', 'rt'])
append_inc_lib(remote_shared_dir)

build_service('remote', [remote_lib])
build_service('motors')
build_service('mixer')

