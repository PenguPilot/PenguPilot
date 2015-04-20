/*___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 MS5611 Reader Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <simple_thread.h>
#include <threadsafe_types.h>
#include <scl.h>
#include <msgpack.h>

#include "ms5611_reader.h"
#include "ms5611.h"


#define THREAD_NAME       "ms5611_reader"
#define THREAD_PRIORITY   99


static simple_thread_t thread;
static ms5611_t ms5611;
static void *baro_raw_socket;
static msgpack_sbuffer *msgpack_buf;
static msgpack_packer *pk;
 

SIMPLE_THREAD_BEGIN(thread_func)
{   
   SIMPLE_THREAD_LOOP_BEGIN
   {
      int status = ms5611_measure(&ms5611);
      msgpack_sbuffer_clear(msgpack_buf);
      if (status == 0)
         PACKF(ms5611.c_a);
      else
         PACKI(status);
      scl_copy_send_dynamic(baro_raw_socket, msgpack_buf->data, msgpack_buf->size);
      msleep(10);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int ms5611_reader_init(i2c_bus_t *bus)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   ms5611_init(&ms5611, bus, MS5611_OSR4096, MS5611_OSR4096);
   baro_raw_socket = scl_get_socket("baro_raw", "pub");
   THROW_IF(baro_raw_socket == NULL, -EIO);

   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
 
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   THROW_END();
}

