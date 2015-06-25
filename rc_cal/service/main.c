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
  
 Remote Control Calibration Service Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <msgpack.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <logger.h>
#include <msgpack_reader.h>
#include <service.h>
#include <pp_prio.h>

#include "../shared/rc_cal.h"
#include "channels.h"



SERVICE_MAIN_BEGIN("rc_cal", PP_PRIO_1)
{
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* initialize SCL: */
   void *rc_raw_socket = scl_get_socket("rc_raw", "sub");
   THROW_IF(rc_raw_socket == NULL, -EIO);
   void *rc_socket = scl_get_socket("rc", "pub");
   THROW_IF(rc_socket == NULL, -EIO);

   /* init channel mapping: */
   LOG(LL_INFO, "loading channels configuration");
   THROW_ON_ERR(channels_init());

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(rc_raw)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         float channels[MAX_CHANNELS];
         int n_channels = root.via.array.size - 1;
         int valid = root.via.array.ptr[0].via.i64;
         FOR_N(i, n_channels)
            if (i < MAX_CHANNELS)
               channels[i] = root.via.array.ptr[1 + i].via.dec;
          
         /* apply permutation and calibration */
         float cal_channels[PP_MAX_CHANNELS];
         channels_update(cal_channels, channels);

         /* send the channels: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, PP_MAX_CHANNELS + 1);
         PACKI(valid);    /* index 0: valid */
         PACKFV(cal_channels, PP_MAX_CHANNELS); /* index 1, .. : channels */
         scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

