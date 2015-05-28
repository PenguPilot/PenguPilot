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
  
 MAG Calibration Service Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <msgpack.h>

#include <scl.h>
#include <service.h>
#include <msgpack_reader.h>

#include "acc_cal.h"


static void *acc_raw_socket = NULL;
static void *acc_cal_socket = NULL;


SERVICE_MAIN_BEGIN("acc_cal", 99)
{ 
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   acc_raw_socket = scl_get_socket("acc_raw", "sub");
   THROW_IF(acc_raw_socket == NULL, -EIO);
   acc_cal_socket = scl_get_socket("acc_cal", "pub");
   THROW_IF(acc_cal_socket == NULL, -EIO);

   /* init calibration data: */
   acc_cal_init();

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(acc_raw)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         vec3_t acc;
         vec3_init(&acc);
         FOR_N(i, 3)
            acc.ve[i] = root.via.array.ptr[i].via.dec;
         acc_cal_apply(&acc);
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(acc.ve, 3);
         scl_copy_send_dynamic(acc_cal_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END;
}
SERVICE_MAIN_END

