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
  
 I2CXL Reader Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <simple_thread.h>
#include <threadsafe_types.h>
#include <scl.h>
#include <msgpack.h>

#include <util.h>
#include "i2cxl_reader.h"
#include "i2cxl.h"


#define THREAD_NAME       "i2cxl_reader"
#define THREAD_PRIORITY   99


static simple_thread_t thread;
static i2cxl_t i2cxl;
static void *ultra_raw_socket;
static msgpack_sbuffer *msgpack_buf;
static msgpack_packer *pk;
 

SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      float alt;
      int status = i2cxl_read(&i2cxl, &alt);
      msgpack_sbuffer_clear(msgpack_buf);
      if (status == 0)
         PACKF(alt);
      else
         PACKI(status);
      scl_copy_send_dynamic(ultra_raw_socket, msgpack_buf->data, msgpack_buf->size);
      msleep(30);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int i2cxl_reader_init(i2c_bus_t *bus)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   i2cxl_init(&i2cxl, bus);
   ultra_raw_socket = scl_get_socket("ultra_raw", "pub");
   THROW_IF(ultra_raw_socket == NULL, -EIO);
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   THROW_END();
}

