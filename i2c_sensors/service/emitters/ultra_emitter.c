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

#define FILTER_SIZE 4

#include <simple_thread.h>
#include <threadsafe_types.h>
#include <scl.h>
#include <msgpack.h>
#include <util.h>
#include <pp_prio.h>
#include <medianFilter.c>

#include "../platform/platform.h"
#include "ultra_emitter.h"

static simple_thread_t thread;
static void *ultra_raw_socket;

//median filter variables
int filterSize = FILTER_SIZE, filterPos;
int filterWindow[FILTER_SIZE]; 

SIMPLE_THREAD_BEGIN(thread_func)
{
   MSGPACK_PACKER_DECL_INFUNC();
   SIMPLE_THREAD_LOOP_BEGIN
   {
      msleep(30);
      float altitude;
      int status = platform_read_ultra(&altitude);
      //apply median filter
      altitude = filter_upd(filterWindow, filterSize, &filterPos, (int)(altitude*100)) / 100;
      msgpack_sbuffer_clear(msgpack_buf);
      if (status == 0)
         PACKF(altitude);
      else
         continue;
      scl_copy_send_dynamic(ultra_raw_socket, msgpack_buf->data, msgpack_buf->size);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int ultra_emitter_start(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   //init median filter
   filter_init(filterWindow, filterSize, &filterPos);
   ultra_raw_socket = scl_get_socket("ultra_raw", "pub");
   THROW_IF(ultra_raw_socket == NULL, -EIO);
   simple_thread_start(&thread, thread_func, "ultra_emitter", PP_PRIO_1, NULL);
   THROW_END();
}

