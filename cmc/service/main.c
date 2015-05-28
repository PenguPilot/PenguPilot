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
  
 MAG Current Calibration Service Implementation

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
#include <threadsafe_types.h>
#include <msgpack_reader.h>


#include "cmc.h"


/* current reader thread: */
tsfloat_t current;
MSGPACK_READER_BEGIN(current_reader)
   MSGPACK_READER_LOOP_BEGIN(current_reader)
   tsfloat_set(&current, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN("cmc", 99)
{
   tsfloat_init(&current, 0.0f);

   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   void *mag_adc_cal_socket = scl_get_socket("mag_adc_cal", "sub");
   THROW_IF(mag_adc_cal_socket == NULL, -EIO);
   void *marg_cal_socket = scl_get_socket("mag_cal", "pub");
   THROW_IF(marg_cal_socket == NULL, -EIO);
   
   /* start current reader: */
   MSGPACK_READER_START(current_reader, "current", 99);
 
   /* init calibration data: */
   cmc_init();
 
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(mag_adc_cal)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         vec3_t mag;
         vec3_init(&mag);
         FOR_N(i, 3)
            mag.ve[i] = root.via.array.ptr[i].via.dec;
         cmc_apply(&mag, tsfloat_get(&current));
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(mag.ve, 3);
         scl_copy_send_dynamic(marg_cal_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

