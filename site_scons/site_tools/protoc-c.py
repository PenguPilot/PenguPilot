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
  
 Protoc-c Builder

 Copyright (C) 2011 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2011 Scott Stafford

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


__author__ = "Scott Stafford"

import SCons.Action
import SCons.Builder
import SCons.Defaults
import SCons.Node.FS
import SCons.Util

from SCons.Script import File, Dir

import os.path

protoccs = 'protoc-c'

ProtocAction = SCons.Action.Action('$PROTOCCCOM', '$PROTOCCCOMSTR')
def ProtocEmitter(target, source, env):
    dirOfCallingSConscript = Dir('.').srcnode()
    env.Prepend(PROTOCPROTOPATH = dirOfCallingSConscript.path)
    
    source_with_corrected_path = []
    for src in source:
        commonprefix = os.path.commonprefix([dirOfCallingSConscript.path, src.srcnode().path])
        if len(commonprefix)>0:
            source_with_corrected_path.append( src.srcnode().path[len(commonprefix + os.sep):] )
        else:
            source_with_corrected_path.append( src.srcnode().path )
        
    source = source_with_corrected_path
    
    for src in source:
        modulename = os.path.splitext(src)[0]

        if env['PROTOCCOUTDIR']:
            base = os.path.join(env['PROTOCCOUTDIR'] , modulename)
            target.append( base + '.pb-c.h' )
            target.append( base + '.pb-c.c' )

    return target, source


ProtocBuilder = SCons.Builder.Builder(action = ProtocAction,
                                   emitter = ProtocEmitter,
                                   srcsuffix = '$PROTOCCSRCSUFFIX')

def generate(env):
    """Add Builders and construction variables for protoc to an Environment."""
    try:
        bld = env['BUILDERS']['ProtocC']
    except KeyError:
        bld = ProtocBuilder
        env['BUILDERS']['ProtocC'] = bld
        
    env['PROTOCC']        = env.Detect(protoccs) or 'protoc-c'
    env['PROTOCCFLAGS']   = SCons.Util.CLVar('')
    env['PROTOCCPROTOPATH'] = SCons.Util.CLVar('')
    env['PROTOCCCOM']     = '$PROTOCC ${["-I%s"%x for x in PROTOCCPROTOPATH]} $PROTOCCFLAGS ${PROTOCCOUTDIR and ("--c_out="+PROTOCCOUTDIR) or ""} ${SOURCES}'
    env['PROTOCCSRCSUFFIX']  = '.proto'
    env['PROTOCCOUTDIR'] = '.'
        

def exists(env):
    return env.Detect(protoccs)
