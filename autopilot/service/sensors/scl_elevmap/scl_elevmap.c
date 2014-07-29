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
  
 SCL Power Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <unistd.h>

#include <util.h>
#include <simple_thread.h>
#include <threadsafe_types.h>
#include <msgpack.h>
#include <scl.h>

#include "scl_elevmap.h"
#include "../../util/logger/logger.h"


#define THREAD_PRIORITY 98


static simple_thread_t thread;
static tsfloat_t elevation;
static void *scl_socket;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      char buffer[128];
      int ret = scl_recv_static(scl_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            assert(root.via.array.size == 1);
            float elev = root.via.array.ptr[0].via.dec;
            tsfloat_set(&elevation, elev);
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         sleep(1);
         LOG(LL_ERROR, "could not read elevation");
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int scl_elevmap_init(void)
{
   THROW_BEGIN();
   scl_socket = scl_get_socket("elev");
   THROW_IF(scl_socket == NULL, -ENODEV);
   tsfloat_init(&elevation, 0.0f);
   simple_thread_start(&thread, thread_func, "elevmap_reader", THREAD_PRIORITY, NULL);
   THROW_END();
}


float scl_elevmap_get(void)
{
   return tsfloat_get(&elevation);
}

