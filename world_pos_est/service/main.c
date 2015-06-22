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
  
 World Position Estimator Service Implementation

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
#include <pthread.h>

#include <scl.h>
#include <service.h>
#include <msgpack_reader.h>
#include <interval.h>
#include <threadsafe_types.h>

#include "pos.h"


tsfloat_t ultra;
MSGPACK_READER_BEGIN(ultra_reader)
   MSGPACK_READER_LOOP_BEGIN(ultra_reader)
      tsfloat_set(&ultra, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

tsfloat_t baro;
MSGPACK_READER_BEGIN(baro_reader)
   MSGPACK_READER_LOOP_BEGIN(baro_reader)
      /* array 0: altitude, array 1: temperature */
      tsfloat_set(&baro, root.via.array.ptr[0].via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END



SERVICE_MAIN_BEGIN("world_pos_est", 99)
{ 
   tsfloat_init(&ultra, 0.2);
   pos_init();
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   void *acc_world_hp_socket = scl_get_socket("acc_world_hp", "sub");
   THROW_IF(acc_world_hp_socket == NULL, -EIO);
   void *world_pos_est_socket = scl_get_socket("world_pos_est", "pub");
   THROW_IF(world_pos_est_socket == NULL, -EIO);

   MSGPACK_READER_START(ultra_reader, "ultra_raw", 99, "sub");
   MSGPACK_READER_START(baro_reader, "baro_raw", 99, "sub");
 
   interval_t interval;
   interval_init(&interval);

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(acc_world_hp)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         pos_t pos;
         pos_in_t pos_in; /* contains: dt, ultra_u, baro_u, pos_n, pos_e, speed_n, speed_e, acc */
         vec3_init(&pos_in.acc);
         pos_in.dt = interval_measure(&interval);
         pos_in.ultra_u = tsfloat_get(&ultra);
         pos_in.baro_u = tsfloat_get(&baro);
         /*
   float dt;
   float ultra_u;
   float baro_u;
   float pos_n;
   float pos_e;
   float speed_n;
   float speed_e;
   vec3_t acc;
          *
          * */
         FOR_N(i, 3)
            pos_in.acc.ve[i] = root.via.array.ptr[i].via.dec;
         
         /* run position estimate: */
         pos_update(&pos, &pos_in);
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 4);
         PACKF(pos.ultra_u.pos);
         PACKF(pos.ultra_u.speed);
         PACKF(pos.baro_u.pos);
         PACKF(pos.baro_u.speed);
         scl_copy_send_dynamic(world_pos_est_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END;
}
SERVICE_MAIN_END

