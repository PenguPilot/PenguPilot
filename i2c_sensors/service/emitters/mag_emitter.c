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
  
 Magnetometer Emitter Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <errno.h>

#include <util.h>
#include <simple_thread.h>
#include <threadsafe_types.h>
#include <msgpack.h>
#include <scl.h>
#include <math/vec3.h>
#include <pp_prio.h>

#include "../platform/platform.h"
#include "mag_emitter.h"


static simple_thread_t thread;
static void *mag_raw_socket;
 

SIMPLE_THREAD_BEGIN(thread_func)
{
   MSGPACK_PACKER_DECL_INFUNC();
   SIMPLE_THREAD_LOOP_BEGIN
   {
      vec3_t vec;
      vec3_init(&vec);
      msgpack_sbuffer_clear(msgpack_buf);
      int ret = platform_read_mag(&vec);
      if (ret == 0)
      {
         msgpack_pack_array(pk, 3);
         PACKFV(vec.ve, 3);
      }
      else
         continue;
      scl_copy_send_dynamic(mag_raw_socket, msgpack_buf->data, msgpack_buf->size);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int mag_emitter_start(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   mag_raw_socket = scl_get_socket("mag_raw", "pub");
   THROW_IF(mag_raw_socket == NULL, -EIO);
   simple_thread_start(&thread, thread_func, "mag_emitter", PP_PRIO_1, NULL);
   THROW_END();
}

