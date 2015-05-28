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

#include "../platform/platform.h"
#include "ultra_emitter.h"


#define THREAD_NAME       "ultra_emitter"
#define THREAD_PRIORITY   99


static simple_thread_t thread;
static void *ultra_raw_socket;
MSGPACK_PACKER_DECL;
 

SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      float altitude;
      int status = platform_read_ultra(&altitude);
      msgpack_sbuffer_clear(msgpack_buf);
      if (status == 0)
         PACKF(altitude);
      else
         PACKI(status);
      scl_copy_send_dynamic(ultra_raw_socket, msgpack_buf->data, msgpack_buf->size);
      msleep(30);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int ultra_emitter_start(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   ultra_raw_socket = scl_get_socket("ultra_raw", "pub");
   THROW_IF(ultra_raw_socket == NULL, -EIO);
   MSGPACK_PACKER_INIT();
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   THROW_END();
}

