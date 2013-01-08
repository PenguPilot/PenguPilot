#!/bin/bash
#  __________________________________
# |       _         _         _      |
# |     _( )__    _( )__    _( )__   |
# |   _|     _| _|     _| _|     _|  |
# |  (_   S (_ (_   C (_ (_   L (_   |
# |    |_( )__|  |_( )__|  |_( )__|  |
# |                                  |
# | Signaling and Communication Link |
# |__________________________________|
#
# SCL Cache Generator Script
#
# Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details. */


ARGS="$@"

OUTPUT_DIR=$HOME/.SCL
if [ ! -d $OUTPUT_DIR ]; then
   mkdir -p $OUTPUT_DIR
fi

if [ ! -f $SCL_CONFIG ]; then
   echo "system specification file not found, aborting"
   exit 2
fi
SYS_CONFIG=$SCL_CONFIG

SYS_CONFIG_FILE_NAME=system.yaml
SCL_SPEC_GENERATOR_SCRIPT=scl_spec_generator.py

SCL_SPEC=$OUTPUT_DIR/scl.yaml
SCL_SPEC_MD5=$SCL_SPEC.md5
SYS_CONFIG_MD5=$OUTPUT_DIR/$SYS_CONFIG_FILE_NAME.md5
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCL_SPEC_GENERATOR=$DIR/$SCL_SPEC_GENERATOR_SCRIPT
SCL_SPEC_GENERATOR_MD5=$OUTPUT_DIR/$SCL_SPEC_GENERATOR_SCRIPT.md5
SCL_SPEC_GENERATOR_ARGS=$OUTPUT_DIR/$SCL_SPEC_GENERATOR_SCRIPT.args
SCL_SPEC_GENERATOR_ARGS_MD5=$SCL_SPEC_GENERATOR_ARGS.md5


BASES="SCL_SPEC SYS_CONFIG SCL_SPEC_GENERATOR SCL_SPEC_GENERATOR_ARGS"


function generate
{
   python $SCL_SPEC_GENERATOR $ARGS > $SCL_SPEC < $SYS_CONFIG
   if [ $? == 0 ]; then
      for base in $BASES
      do
         digest=${base}_MD5
         md5sum ${!base} > ${!digest}
      done
   else
      echo generator script failed with code $?
      rm -f $SCL_SPEC $SCL_SPEC_GENERATOR_ARGS
      for base in $BASES
      do
         digest=${base}_MD5
         rm -f ${!digest}
      done
      exit 3
   fi
}


# run the md5-checks and regenerate, if required:

echo $ARGS > $SCL_SPEC_GENERATOR_ARGS

for base in $BASES
do
digest=${base}_MD5
md5sum --quiet --check ${!digest} &> /dev/null < ${!base}
if [ $? != 0 ]; then 
   generate
   break
fi
done

